import asyncio
import inspect
import uuid
from contextlib import AsyncExitStack, asynccontextmanager
from functools import wraps

import uvicorn
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.sdp import SessionDescription
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from .. import logger
from ..utils.codecs import get_video_codecs_from_sdp


class WebRTCServer:
    """
    A reusable WebRTC server that allows customization of video tracks,
    data channel message handlers, and a shared state object per peer.
    """

    def __init__(
        self,
        host="0.0.0.0",
        port=8080,
        video_track_factory=None,
        datachannel_handlers=None,
        state_factory=None,
        server_data_channels=None,
    ):
        """
        Initializes the WebRTC Server.

        Args:
            host (str, optional): The host to bind the server to. Defaults to "0.0.0.0".
            port (int, optional): The port to run the server on. Defaults to 8080.
            video_track_factory (callable, optional): A function or class that, when called,
                returns a new instance of a MediaStreamTrack. It will receive a `state` object
                as a keyword argument if its signature includes `state` or `**kwargs`.
            datachannel_handlers (dict, optional): A dictionary mappping data channel labels
                (str) to callback functions. Callbacks will receive a `state` object as a
                keyword argument if their signature includes `state` or `**kwargs`.
            state_factory (callable, optional): A function or class that, when called, returns
                a new state object for the peer connection.
            server_data_channels (iterable[str], optional): Data channel labels to be created
                by the server proactively (e.g., control channels).
        """
        self.host = host
        self.port = port
        self.state_factory = state_factory
        self.app = FastAPI(lifespan=self.lifespan)
        self.pcs = set()  # global storage for peer connection(s)
        self._peer_context = {}  # pc -> {"state": state, "channels": {label: channel}}
        self._extra_lifespans = []  # additional async context managers to run around lifespan
        self._server_data_channels = list(server_data_channels or [])

        # Wrap factories and handlers to manage state passing and async execution
        self._video_track_factory = self._wrap_callable(video_track_factory)
        self._datachannel_handlers = {
            label: self._wrap_callable(handler)
            for label, handler in (datachannel_handlers or {}).items()
        }

        self.app.post("/offer")(self._create_offer_handler)  # WebRTC signal endpoint

    def _wrap_callable(self, func):
        """
        Wraps a user-provided callable (factory or handler) to standardize its
        execution.

        This wrapper performs two main functions:
        1.  **State Injection**: It inspects the callable's signature once. If the
            callable can accept a `state` keyword argument (i.e., it has a
            `state` parameter or `**kwargs`), the wrapper will pass the
            peer-specific state object to it. This is done at initialization
            to avoid repeated, costly `inspect` calls in the hot path.
        2.  **Async Handling**: It ensures that both synchronous and asynchronous
            callables are handled correctly by returning an `async` wrapper that
            `await`s the original function if it's a coroutine.

        Args:
            func (callable): The function or callable to wrap.

        Returns:
            An async wrapper function that normalizes the callable's execution.
            Returns None if the input is None.
        """
        if func is None:
            return None

        sig = inspect.signature(func)
        params = sig.parameters
        accepts_kwargs = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values())
        has_state = "state" in params or accepts_kwargs
        has_channel = "channel" in params or accepts_kwargs
        accepted_names = set(params.keys())
        is_async = asyncio.iscoroutinefunction(func)

        @wraps(func)
        async def wrapper(*args, **kwargs):
            state = kwargs.pop("state", None)
            channel = kwargs.pop("channel", None)
            # Trim kwargs that aren't accepted unless the callable has **kwargs
            call_args = (
                kwargs
                if accepts_kwargs
                else {k: v for k, v in kwargs.items() if k in accepted_names}
            )

            if has_state:
                call_args["state"] = state
            if has_channel:
                call_args["channel"] = channel

            if is_async:
                return await func(*args, **call_args)
            else:
                return func(*args, **call_args)

        return wrapper

    def add_lifespan_context(self, ctx):
        """
        Register an async context manager (or factory) to be composed with the server lifespan.

        Args:
            ctx: Either an async context manager instance or a callable that accepts
                 the FastAPI app and returns an async context manager.
        """
        self._extra_lifespans.append(ctx)

    def iter_peer_states(self):
        """Yield the state object for each active peer connection."""
        for ctx in self._peer_context.values():
            state = ctx.get("state")
            if state is not None:
                yield state

    def find_state(self, predicate):
        """
        Return the first peer state for which predicate(state) is True, else None.
        """
        for state in self.iter_peer_states():
            if predicate(state):
                return state
        return None

    def _attach_channel_handlers(self, pc, channel, state, pc_id):
        """
        Register message/close handlers for a data channel and track it.
        """
        label = channel.label
        logger.info(f"{pc_id}: Data channel '{label}' created.")
        peer_ctx = self._peer_context.get(pc)
        if peer_ctx is not None:
            peer_ctx["channels"][label] = channel

        if label in self._datachannel_handlers:
            handler = self._datachannel_handlers[label]

            @channel.on("message")
            async def on_message(message):
                logger.debug(f"{pc_id}: Message on '{label}': {message}")
                await handler(message=message, state=state, channel=channel)
        else:
            logger.warning(f"{pc_id}: No handler registered for data channel '{label}'.")

        @channel.on("close")
        def on_close():
            peer_ctx = self._peer_context.get(pc)
            if peer_ctx:
                peer_ctx["channels"].pop(label, None)
                logger.info(f"{pc_id}: Data channel '{label}' closed and removed.")

    @asynccontextmanager
    async def lifespan(self, app: FastAPI):
        async with AsyncExitStack() as stack:
            for ctx in self._extra_lifespans:
                ctx_obj = ctx(app) if callable(ctx) else ctx
                await stack.enter_async_context(ctx_obj)

            # Startup
            yield

            # Shutdown
            logger.info("Server shutting down, closing all peer connections.")
            # Make a copy of the set to iterate over, as closing pcs modifies the set
            coros = [pc.close() for pc in list(self.pcs)]
            await asyncio.gather(*coros)
            self.pcs.clear()
            self._peer_context.clear()

    async def _create_offer_handler(self, request: Request):
        """
        Handles the SDP offer from the client and returns an SDP answer.
        """
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        # Create a new peer connection
        pc = RTCPeerConnection()
        pc_id = f"PeerConnection({uuid.uuid4()})"
        self.pcs.add(pc)
        logger.info(f"{pc_id}: Created PeerConnection for {request.client.host}")

        # Create a state object for peer connection
        state = None
        if self.state_factory:
            state = self.state_factory()
            logger.info(f"{pc_id}: Created state object: {state}")

        # Create a video track if peer connection requests
        if self._video_track_factory:
            client_video_codecs = get_video_codecs_from_sdp(offer.sdp)
            logger.debug(f"Video codecs available in client: {client_video_codecs}")
            parsed_offer = SessionDescription.parse(offer.sdp)
            logger.debug(f"Client {parsed_offer.media=}")
            if any(m.kind == "video" and m.port != 0 for m in parsed_offer.media):
                logger.info(f"{pc_id}: Client wants video, creating track.")
                video_track = await self._video_track_factory(state=state)
                pc.addTrack(video_track)
            else:
                logger.info(f"{pc_id}: Client does not want video, not adding track.")
        else:
            logger.warning(f"{pc_id}: No video_track_factory provided.")

        # Track state and channels for this peer
        self._peer_context[pc] = {"state": state, "channels": {}, "id": pc_id}

        # Create server-initiated data channels (e.g., control channels)
        for label in self._server_data_channels:
            try:
                channel = pc.createDataChannel(label)
                self._attach_channel_handlers(pc, channel, state, pc_id)
            except Exception as e:
                logger.error(f"{pc_id}: Failed to create server data channel '{label}': {e}")

        # Create a data channel handler
        @pc.on("datachannel")
        def on_datachannel(channel):
            self._attach_channel_handlers(pc, channel, state, pc_id)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            logger.info(f"{pc_id}: Connection state is {pc.connectionState}")
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                self.pcs.discard(pc)
                self._peer_context.pop(pc, None)
                logger.info(f"{pc_id}: Cleaned up.")

        try:
            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            server_video_codecs = get_video_codecs_from_sdp(answer.sdp)
            logger.debug(f"{pc_id}: Video codecs available in server: {server_video_codecs}")
            parsed_offer = SessionDescription.parse(answer.sdp)
            logger.debug(f"Server {parsed_offer.media=}")
            await pc.setLocalDescription(answer)

        except Exception as e:
            logger.error(f"{pc_id}: Error during offer/answer exchange: {e}")
            await pc.close()
            self.pcs.discard(pc)
            self._peer_context.pop(pc, None)
            return JSONResponse(status_code=500, content={"error": str(e)})

        # Return the answer to the client
        return JSONResponse(
            content={"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        )

    def send_to_datachannel(self, label, message, state=None):
        """
        Send a message to any open data channel with the given label.

        If `state` is provided, the send targets only the peer associated with
        that state; otherwise it broadcasts to all peers with the label.

        Returns:
            int: The number of channels the message was sent to.
        """
        deliveries = 0
        for pc in list(self.pcs):
            ctx = self._peer_context.get(pc)
            if ctx is None:
                continue
            if state is not None and ctx["state"] is not state:
                continue

            channel = ctx["channels"].get(label)
            if channel is None:
                continue

            if getattr(channel, "readyState", None) != "open":
                logger.debug(f"{ctx.get('id')}: Data channel '{label}' not open; skipping send.")
                continue

            try:
                channel.send(message)
                deliveries += 1
            except Exception as e:
                logger.error(f"{ctx.get('id')}: Failed to send on '{label}': {e}")
        return deliveries

    def run(self):
        """Starts the web server."""
        uvicorn.run(self.app, host=self.host, port=self.port)

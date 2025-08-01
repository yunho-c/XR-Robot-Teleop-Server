import asyncio
import inspect
import uuid
from contextlib import asynccontextmanager
from functools import wraps

import uvicorn
from aiortc import RTCPeerConnection, RTCSessionDescription
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from .. import logger


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
        """
        self.host = host
        self.port = port
        self.state_factory = state_factory
        self.app = FastAPI(lifespan=self.lifespan)
        self.pcs = set()  # global storage for peer connection(s)

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
        has_state = "state" in sig.parameters or any(
            p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values()
        )
        is_async = asyncio.iscoroutinefunction(func)

        @wraps(func)
        async def wrapper(*args, **kwargs):
            state = kwargs.pop("state", None)
            call_args = kwargs

            if has_state:
                call_args["state"] = state

            if is_async:
                return await func(*args, **call_args)
            else:
                return func(*args, **call_args)

        return wrapper

    @asynccontextmanager
    async def lifespan(self, app: FastAPI):
        # Startup
        yield

        # Shutdown
        logger.info("Server shutting down, closing all peer connections.")
        # Make a copy of the set to iterate over, as closing pcs modifies the set
        coros = [pc.close() for pc in list(self.pcs)]
        await asyncio.gather(*coros)
        self.pcs.clear()

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
            from aiortc.sdp import SessionDescription

            parsed_offer = SessionDescription.parse(offer.sdp)
            if any(m.kind == "video" and m.port != 0 for m in parsed_offer.media):
                logger.info(f"{pc_id}: Client wants video, creating track.")
                video_track = await self._video_track_factory(state=state)
                pc.addTrack(video_track)
            else:
                logger.info(f"{pc_id}: Client does not want video, not adding track.")
        else:
            logger.warning(f"{pc_id}: No video_track_factory provided.")

        # Create a data channel handler
        @pc.on("datachannel")
        def on_datachannel(channel):
            label = channel.label
            logger.info(f"{pc_id}: Data channel '{label}' created.")

            if label in self._datachannel_handlers:
                handler = self._datachannel_handlers[label]

                @channel.on("message")
                async def on_message(message):
                    logger.debug(f"{pc_id}: Message on '{label}': {message}")
                    await handler(message=message, state=state)
            else:
                logger.warning(f"{pc_id}: No handler registered for data channel '{label}'.")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            logger.info(f"{pc_id}: Connection state is {pc.connectionState}")
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                self.pcs.discard(pc)
                logger.info(f"{pc_id}: Cleaned up.")

        try:
            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)

        except Exception as e:
            logger.error(f"{pc_id}: Error during offer/answer exchange: {e}")
            await pc.close()
            self.pcs.discard(pc)
            return JSONResponse(status_code=500, content={"error": str(e)})

        # Return the answer to the client
        return JSONResponse(
            content={"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        )

    def run(self):
        """Starts the web server."""
        uvicorn.run(self.app, host=self.host, port=self.port)

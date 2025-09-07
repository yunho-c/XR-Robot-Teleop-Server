def get_video_codecs_from_sdp(sdp: str) -> str:
    """Parses SDP to extract video codec information."""
    rtpmap_lines = [line.strip() for line in sdp.splitlines() if line.startswith("a=rtpmap:")]
    codec_lines = [
        line
        for line in rtpmap_lines
        if "rtx" not in line and "red" not in line and "ulpfec" not in line
    ]
    return "\n".join(codec_lines)

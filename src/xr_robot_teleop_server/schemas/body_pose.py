import struct

from ..utils.coordinate import convert_unity_to_right_handed_z_up


class Bone:
    """
    A simple data structure to hold the bone data.
    """

    def __init__(
        self,
        id: int,
        position: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ):
        self.id = id
        self.position = position
        self.rotation = rotation

    def __repr__(self):
        return f"Bone(pos={self.position}, rot={self.rotation})"


def deserialize_pose_data(data: bytes, z_up: bool = True) -> list[Bone]:
    """
    Deserializes the binary pose data stream from the Unity client.

    Args:
        data: The raw byte string received from the data channel.

    Returns:
        A list of Bone objects.
    """
    bones = []
    offset = 0

    # The C# BinaryWriter is little-endian by default. The format string '<' specifies this.
    # Format: 1 int (id) + 7 floats (pos/rot) = 8 values
    # '<i' = little-endian integer (4 bytes)
    # '<7f' = 7 little-endian floats (7 * 4 = 28 bytes)
    # Total size per bone: 32 bytes

    try:
        # 1. Read the number of bones (an integer)
        (bone_count,) = struct.unpack_from("<i", data, offset)
        offset += 4

        # 2. Loop for each bone to read its data
        for _ in range(bone_count):
            # Ensure there is enough data left in the buffer
            if offset + 32 > len(data):
                print("Error: Incomplete data buffer for a bone.")
                break

            # 3. Unpack 8 values: id, pos(x,y,z), rot(x,y,z,w)
            bone_data = struct.unpack_from("<i7f", data, offset)
            offset += 32

            bone_id = bone_data[0]
            position = (bone_data[1], bone_data[2], bone_data[3])
            rotation = (bone_data[4], bone_data[5], bone_data[6], bone_data[7])

            # 4. Convert to z-up (FLU) from if user wants
            if z_up:
                position, rotation = convert_unity_to_right_handed_z_up(position, rotation)

            bones.append(Bone(bone_id, position, rotation))

    except struct.error as e:
        print(f"Error deserializing pose data: {e}")

    return bones

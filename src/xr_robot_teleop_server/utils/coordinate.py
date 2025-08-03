def convert_unity_to_right_handed_z_up(
    position: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """
    Converts position and rotation from Unity's left-handed, Y-up coordinate system
    to a right-handed, Z-up coordinate system (+X Forward, +Y Left, +Z Up).
    """
    # Position conversion: Unity (x,y,z) -> (z, -x, y)
    new_position = (position[2], -position[0], position[1])

    # Rotation quaternion conversion: Unity (qx,qy,qz,qw) -> (-qz, qx, -qy, qw)
    qx, qy, qz, qw = rotation
    new_rotation = (-qz, qx, -qy, qw)

    return new_position, new_rotation

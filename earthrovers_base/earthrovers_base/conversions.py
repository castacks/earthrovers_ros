"""Module containing functions to perform various unit and data type
conversions.
"""

def gs_to_ms2(gs: float) -> float:
    """Converts acceleration from g's to m/s^2.

    Args:
        gs (float): The acceleration in g's.

    Returns:
        float: The acceleration in m/s^2.
    """
    # Technically, could compute this on the fly if you knew the altitude.
    # That's a bit overkill for us right now though :D
    GRAVITY_MS2 = 9.80665
    return gs * GRAVITY_MS2

def degs_to_rads(degrees_s: float) -> float:
    """Converts angular velocity from degrees/s to rad/s.

    Args:
        degrees_s (float): The angular velocity in degrees/s.

    Returns:
        float: The angular velocity in rad/s.
    """
    PI = 3.14159
    DEG_IN_PI = 180.0
    return degrees_s * (PI / DEG_IN_PI)

def lsb_to_gauss(lsb: int, lsb_per_gauss: int) -> float:
    """Converts magnetic field strength from LSB to gauss.

    Args:
        lsb (int): The magnetic field strength in LSB.
        lsb_per_gauss (int): The number of LSBs per gauss for the sensor ==
        the sensor's sensitivity/resolution. This is typically computed as the
        range / 2^number_of_bits in the sensor's ADC output.

    Returns:
        float: The magnetic field strength in gauss.
    """
    return lsb * lsb_per_gauss

def gauss_to_tesla(gauss: float) -> float:
    """Converts magnetic field strength from gauss to tesla.

    Args:
        gauss (float): The magnetic field strength in gauss.

    Returns:
        float: The magnetic field strength in tesla.
    """
    gauss_PER_TESLA = 10000
    return gauss * (1.0/gauss_PER_TESLA)

def lsb_to_tesla(lsb: int, lsb_per_gauss: int) -> float:
    """Converts magnetic field strength from LSB to tesla.

    Args:
        lsb (int): The magnetic field strength in LSB.
        lsb_per_gauss (int): The number of LSBs per gauss for the sensor ==
        the sensor's sensitivity/resolution. This is typically computed as the
        range / 2^number_of_bits in the sensor's ADC output.

    Returns:
        float: The magnetic field strength in tesla.
    """
    gauss = lsb_to_gauss(lsb, lsb_per_gauss)
    return gauss_to_tesla(gauss)
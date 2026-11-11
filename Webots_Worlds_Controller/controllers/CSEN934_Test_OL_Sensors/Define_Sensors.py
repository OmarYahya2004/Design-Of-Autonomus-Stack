def init_sensors(driver, timestep):
    """
    Initializes and enables all required Webots sensors.
    Returns a dictionary containing the sensor objects.
    """
    camera = driver.getDevice('front_camera')
    if camera:
        camera.enable(timestep)

    gps = driver.getDevice('gps')
    if gps:
        gps.enable(timestep)

    imu = driver.getDevice('imu')
    if imu:
        imu.enable(timestep)

    gyro = driver.getDevice('gyro')
    if gyro:
        gyro.enable(timestep)

    return {
        'camera': camera,
        'gps': gps,
        'imu': imu,
        'gyro': gyro
    }

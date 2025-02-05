public class Robot
{
    public MecanumDrive drive;
    public IMU imu;
    
    private RevHubOrientationOnRobot orientation = null;

    static final int    DRIVER_MODE_FIELD = 0;
    static final int    DRIVER_MODE_ROBOT = 1;
    public int driver_mode = DRIVER_MODE_ROBOT;

    abstract public void init(HardwareMap hardwareMap)
    {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);   
    }

    protected void setOrientation(LogoFacingDirection logo, UsbFacingDirection usb)
    {
        orientation = new RevHubOrientationOnRobot(logo, usb);
    }
}

public class RobotGeddy
{
    public MecanumDrive drive = new MecanumDrive();
    public DcMotor      arm_hang_motor = null;
    public DcMotor      arm_extend_motor = null;
    public DcMotor      shoulder_motor = null;
    public Servo       gripper_servo = null;
    public CRServo     wrist_servo = null;
    public TouchSensor shoulder_sensor = null;
    public TouchSensor arm_sensor = null;
    static final double INTAKE_SPEED = 1.0;
    static final double OUTTAKE_SPEED = -1.0;
    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    static final double SPEED_INCREMENT = 0.1;
    public double current_speed = .5;
    
    
    public static final double GRIPPER_OPEN = 0.70;
    public static final double GRIPPER_CLOSED = 1.0;
    public static final int GRIPPER_STATUS_OPEN = 0;
    public static final int GRIPPER_STATUS_CLOSED = 1;
    public int current_gripper_status = GRIPPER_STATUS_CLOSED;

    public boolean ARM_IS_MOVING = false;    
    public static final int ARM_STATUS_CLOSED = 0;
    public static final int ARM_STATUS_PICKUP = 1;
    public static final int ARM_STATUS_LOW_BASKET = 2;
    public static final int ARM_STATUS_HIGH_BASKET = 3;
    public static final int ARM_STATUS_HIGH_CHAMBER = 4;
    public int current_arm_status = ARM_STATUS_CLOSED;
    public String current_arm_status_name = "ARM_STATUS_CLOSED"; 
    public static final int ARM_INVALID_POS = -6996;
    public int current_target_position = ARM_INVALID_POS;
    public boolean override_checking = false;
    
    public static final int HIGH_BASKET_LOCATION    = 4600;//4500 in auto, used to be 4700
    public static final int LOW_BASKET_LOCATION     = 2500;
    public static final int PICKUP_LOCATION         = 1800; //1300 in auto
    public static final int HIGH_CHAMBER_LOCATION   = 1750;
    
    public static final int ARM_EXTEND_MAX = 2549;
    
    public static final int SHOULDER_LOW_POSITION = 600;
    
    static final int    DRIVER_MODE_FIELD = 0;
    static final int    DRIVER_MODE_ROBOT = 1;
    public int driver_mode = DRIVER_MODE_ROBOT;
    
    public int counter = 0;
   
    private ElapsedTime runtime = new ElapsedTime();
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection  IMU_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_FACING_DIRECTION);
 
    public RobotGeddy()
    {
        setOrientation(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    }
    public void init(HardwareMap hardwareMap) {
 
        base.init(hardwareMap);
        drive.init(hardwareMap);

        gripper_servo = hardwareMap.get(Servo.class, "gripper_servo");
        wrist_servo = hardwareMap.get(CRServo.class, "wrist_servo");

        arm_extend_motor = hardwareMap.get(DcMotor.class, "arm_extend_motor");
        arm_extend_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_extend_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_extend_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        arm_sensor = hardwareMap.get(TouchSensor.class, "arm_sensor");

        arm_hang_motor = hardwareMap.get(DcMotor.class, "arm_hang_motor");
        arm_hang_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulder_motor = hardwareMap.get(DcMotor.class, "shoulder_motor");
        shoulder_motor.setDirection(DcMotor.Direction.REVERSE);
        shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        shoulder_sensor = hardwareMap.get(TouchSensor.class, "shoulder_sensor");
    }
    public void reset_arm_extend()
    {
        arm_extend_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_extend_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void reset_arm_hang()
    {
        arm_hang_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void reset_shoulder()
    {
        shoulder_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void reset_shoulder(DcMotor.RunMode mode)
    {
        reset_shoulder();
        shoulder_motor.setMode(mode);
    }
    public void stop_all_move()
    {
        drive.stop();
        arm_extend_motor.setPower(0);
        shoulder_motor.setPower(0);
    }
    public void ExtendArm(double speed, double pos, double timeoutS)
    {
            ARM_IS_MOVING = true;
            current_target_position = (int)pos;

            int new_distance = arm_extend_motor.getCurrentPosition();
            speed = Math.abs(speed);
        
            arm_extend_motor.setTargetPosition(current_target_position);
            // Turn On RUN_TO_POSITION
            arm_extend_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            arm_extend_motor.setPower(speed);
    }
    public boolean IsShoulderLow()
    {
        if (override_checking)
        {
            return false;
        }
        else
        {
            return (shoulder_motor.getCurrentPosition() < SHOULDER_LOW_POSITION);
        }
    }
    
    public void GripperOpen()
    {
         gripper_servo.setPosition(GRIPPER_OPEN);
         current_gripper_status = GRIPPER_STATUS_OPEN;
    }
    public void GripperClosed()
    {
         gripper_servo.setPosition(GRIPPER_CLOSED);
         current_gripper_status = GRIPPER_STATUS_CLOSED;
    }
    public void SetArmPosition(int pos)
    {
        switch(pos)
        {
            case ARM_STATUS_CLOSED:
            {
                ExtendArm(1.0, 0,3);
                current_arm_status = ARM_STATUS_CLOSED;
                current_arm_status_name = "ARM_STATUS_CLOSED";
                break;
            }
            case ARM_STATUS_PICKUP:
            {
                ExtendArm(1.0, PICKUP_LOCATION, 3);
                current_arm_status = ARM_STATUS_PICKUP ;
                current_arm_status_name = "ARM_STATUS_PICKUP";
                break;
            }
            case ARM_STATUS_LOW_BASKET:
            {
                ExtendArm(1.0, LOW_BASKET_LOCATION, 4);
                current_arm_status = ARM_STATUS_LOW_BASKET;
                current_arm_status_name = "ARM_STATUS_LOW_BASKET";
                break;
            }
            case ARM_STATUS_HIGH_BASKET:
            {
                ExtendArm(1.0, HIGH_BASKET_LOCATION, 5);
                current_arm_status = ARM_STATUS_HIGH_BASKET;
                current_arm_status_name = "ARM_STATUS_HIGH_BASKET";
                break;
            }
            case ARM_STATUS_HIGH_CHAMBER:
            {
                ExtendArm(1.0, HIGH_CHAMBER_LOCATION, 7);
                current_arm_status = ARM_STATUS_HIGH_CHAMBER;
                current_arm_status_name = "ARM_STATUS_HIGH_CHAMBER";
                break;
            }     
        }
    }
}
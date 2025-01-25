import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrive
{
    private DcMotor      left_front_drive  = null;
    private DcMotor      left_rear_drive   = null;
    private DcMotor      right_front_drive = null;
    private DcMotor      right_rear_drive  = null;

    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    private double speedAdjustment = MAX_MOVE_SPEED;

    public void init(HardwareMap hardwareMap) {

        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_rear_drive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        right_front_drive  = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_rear_drive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        
        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_rear_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_rear_drive.setDirection(DcMotor.Direction.FORWARD);
        
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(DcMotor.RunMode mode)
    {
        left_front_drive.setMode(mode);
        left_rear_drive.setMode(mode);
        right_front_drive.setMode(mode);
        right_rear_drive.setMode(mode);      
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode)
    {
        left_front_drive.setZeroPowerBehavior(mode);
        left_rear_drive.setZeroPowerBehavior(mode);
        right_front_drive.setZeroPowerBehavior(mode);
        right_rear_drive.setZeroPowerBehavior(mode);      
    }

    public void setMaxSpeed(double speed)
    {
        speedAdjustment = Range.clip(speed, MIN_MOVE_SPEED, MAX_MOVE_SPEED);
    }

    public double getMaxSpeed()
    {
        return speedAdjustment;
    }

    public void setPower(double left_front, double right_front, double left_back, double right_back)
    {
        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double currentMaxSpeed = MAX_MOVE_SPEED;
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(left_front));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(right_front));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(left_back));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(right_back));

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        left_front /= currentMaxSpeed;
        right_front /= currentMaxSpeed;
        left_back /= currentMaxSpeed;
        right_back /= currentMaxSpeed;

        //flat adjustment, easy way to lower power without changing lots of code
        left_front *= speedAdjustment;
        right_front *= speedAdjustment;
        left_back *= speedAdjustment;
        right_back *= speedAdjustment;

        setRawPower(left_front, right_front, left_back, right_back);
    }

    public void setRawPower(double left_front, double right_front, double left_back, double right_back)
    {
        left_front_drive.setPower(left_front);
        left_rear_drive.setPower(left_back);   
        right_front_drive.setPower(right_front);
        right_rear_drive.setPower(right_back); 
    }
    /*
    * If we had a gyro and wanted to do field-oriented control, here
    * is where we would implement it.
    *
    * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
    * coordinate (strafe, drive), and we just rotate it by the gyro
    * reading minus the offset that we read in the init() method.
    * Some rough pseudocode demonstrating:
    *
    * if Field Oriented Control:
    *     get gyro heading
    *     subtract initial offset from heading
    *     convert heading to radians (if necessary)
    *     new strafe = strafe * cos(heading) - drive * sin(heading)
    *     new drive  = strafe * sin(heading) + drive * cos(heading)
    *
    * If you want more understanding on where these rotation formulas come
    * from, refer to
    * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
    */
    public void move(double drive, double strafe, double twist)
    {
        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
        };
        // apply the calculated values to the motors.
        setPower(speeds[0], speeds[1], speeds[2], speeds[3]);
    }

    void stop()
    {
        setRawPower(0,0,0,0);
    }
}

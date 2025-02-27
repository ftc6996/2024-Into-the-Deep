package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Gripper
{
    public Servo _servo = null;
    
    private double _open_position = 0.70;
    private double _closed_position = 1.0;

    public static final int GRIPPER_STATUS_UNKNOWN = -1;
    public static final int GRIPPER_STATUS_OPEN = 0;
    public static final int GRIPPER_STATUS_CLOSED = 1;
    private int _status = GRIPPER_STATUS_UNKNOWN;
    private String _statusString = "UNKNOWN";
    private String _deviceName = "gripper_servo";

    public Gripper(HardwareMap hardwareMap, String deviceName)
    {
        _deviceName = deviceName;
        init(hardwareMap);
    }

    public Gripper(HardwareMap hardwareMap, String deviceName, double open, double closed)
    {
        _deviceName = deviceName;
        init(hardwareMap);
        //_servo.setDirection(Servo.Direction.REVERSE);
        setOpenPosition(open);
        setClosedPosition(closed);
    }

    public void init(HardwareMap hardwareMap)
    {
        _servo = hardwareMap.get(Servo.class, _deviceName);
        setStatus(GRIPPER_STATUS_UNKNOWN);
        
    }
    public void init(HardwareMap hardwareMap, double open, double closed)
    {
        init(hardwareMap);
        setOpenPosition(open);
        setClosedPosition(closed);
    }

    public void setDirection(Servo.Direction direction)
    {
        _servo.setDirection(direction);
    }

    public void setOpenPosition(double position)
    {
        if (position >= 0 && position <= 1.0)
        {
           _open_position = Range.clip(position, 0.0, 1.0);
        }
    }
    public void setClosedPosition(double position)
    {
        if (position >= 0 && position <= 1.0)
        {
            _closed_position = Range.clip(position, 0.0, 1.0);
        }
    }
    public void open()
    {
         _servo.setPosition(_open_position);
         setStatus(GRIPPER_STATUS_OPEN);
    }
    public void closed()
    {
         _servo.setPosition(_closed_position);
         setStatus(GRIPPER_STATUS_CLOSED);
    }
    public int getStatus()
    {
        return _status;
    }
    public String getStatusString()
    {
        return _statusString;
    }
    private void setStatus(int status)
    {
        if (status == GRIPPER_STATUS_OPEN)
        {
            _status = GRIPPER_STATUS_OPEN;
            _statusString = "OPEN";
        }
        else if (status == GRIPPER_STATUS_CLOSED)
        {
            _status = GRIPPER_STATUS_CLOSED;
            _statusString = "CLOSED";
        }
        else
        {
            _status = GRIPPER_STATUS_UNKNOWN;
            _statusString = "UNKNOWN";
        }
    }
}
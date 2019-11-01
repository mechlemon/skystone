package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;




//HELPER CLASS
//this class makes it more convienient to use the IMU. Before running the OpMode, call IMU.initialize.
//At the beginning of the OpMode, call resetHeading. Whenever you need the heading, call getHeading.


public class IMU {


    public enum HeadingAxis{
        ROLL, PITCH, YAW
    }


    //how the IMU outputs data
    private Orientation angles;


    //the imu sensor belonging to this class:
    private BNO055IMU imu;


    //to reset the imu
    private double headingAngleOffset = 0;


    private HeadingAxis headingAxis = HeadingAxis.YAW;



    public IMU(BNO055IMU hardwareMappedIMU) {
        this.imu = hardwareMappedIMU;
    }


    public void initialize() {
        // Set up the parameters for how the IMU is going to give info
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        // Make sure it is configured on robot as IC2 port 0 "Expansion Hub Internal IMU" and named "imu"
        //applies those parameters to imu
        imu.initialize(parameters);
    }


    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(headingAxis == HeadingAxis.ROLL) return angles.thirdAngle - headingAngleOffset;
        if(headingAxis == HeadingAxis.PITCH) return angles.secondAngle - headingAngleOffset;
        if(headingAxis == HeadingAxis.YAW) return angles.firstAngle - headingAngleOffset;
        return 0;
    }

    public HeadingAxis getHeadingAxis(){
        return headingAxis;
    }

    public void setHeadingAxis(IMU.HeadingAxis newHeadingAxis){
        headingAxis = newHeadingAxis;
    }


    public double getNormalizedHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Calculate.normalizeAngle(angles.firstAngle - headingAngleOffset);
    }


    public void resetHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        headingAngleOffset = angles.firstAngle;
    }


    public double getHeadingAngleOffset(){
        return headingAngleOffset;
    }


    public void setHeadingAngleOffset(double offset){
        headingAngleOffset = offset;
    }








}
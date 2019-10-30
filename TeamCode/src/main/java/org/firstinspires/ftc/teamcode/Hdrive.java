package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


//HELPER CLASS-creates an Hdrivetrain

public class Hdrive {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor strafeMotor1;
    public DcMotor strafeMotor2;

    private double leftPower = 0;
    private double rightPower = 0;
    private double strafePower = 0;


    public Hdrive(DcMotor leftMotor, DcMotor rightMotor, DcMotor strafeMotor1, DcMotor strafeMotor2) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.strafeMotor1 = strafeMotor1;
        this.strafeMotor2 = strafeMotor2;
        clear(); //makes sure the drivetrain doesn't move upon initialization
    }


    //call this every loop to make sure the power sent to the motors is updated
    //also does a last clip to make sure power never exceeds 1
    public void execute(){
        leftMotor.setPower(Range.clip(leftPower,-1,1));
        rightMotor.setPower(Range.clip(rightPower,-1,1));
        strafeMotor1.setPower(Range.clip(strafePower,-1,1));
        strafeMotor2.setPower(Range.clip(strafePower,-1,1));
    }




    //THE GETTERS ##################################################


    public double[] getPowers(){ return new double[] {leftPower, rightPower, strafePower}; }
    public double getleftPower(){ return leftPower; }
    public double getrightPower(){ return rightPower; }
    public double getstrafePower(){ return strafePower; }




    //THE SETTERS ##################################################

    public void XYtoPowers(double x, double y, double spin){
        leftPower = y - spin;
        rightPower = y + spin;
        strafePower = x;
    }


    //move the robot in an angle relative to the field
    public void moveGlobalAngle(double desiredAngle, double robotHeading, double power, double spin) {
        double[] cartesianCoords = Calculate.polarToCartesian(power, desiredAngle, false);
        double[] globalVector = Calculate.FOD(cartesianCoords[0], cartesianCoords[1], -robotHeading, false, true);
        double x = globalVector[0];
        double y = globalVector[1];
        XYtoPowers(x, y, spin);
    }


    //move the robot in an angle relative to the robot's heading
    public void moveLocalAngle(double angle, double power, double spin) {
        double x = Math.cos(Math.toRadians(angle)) * power;
        double y = Math.sin(Math.toRadians(angle)) * power;
        XYtoPowers(x, y, spin);
    }


    public void moveGlobalVector(double xComp, double yComp, double heading, double spin) {
        double[] globalVector = Calculate.FOD(xComp, yComp, heading, true, true);
        double x = globalVector[0];
        double y = globalVector[1];
        XYtoPowers(x, y, spin);
    }


    //zeros them out (stops drivetrain if execute() is run)
    public void clear() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        strafeMotor1.setPower(0);
        strafeMotor2.setPower(0);
        execute();
    }


    //stops and closes motors entirely
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        strafeMotor1.setPower(0);
        strafeMotor2.setPower(0);
        execute();
        leftMotor.close();
        rightMotor.close();
        strafeMotor1.close();
        strafeMotor2.close();
    }




    public void setPowers(double leftPower, double rightPower, double strafePower) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.strafePower = strafePower;
    }
    public void setleftPower(double power){ leftPower = power; }
    public void setrightPower(double power){ rightPower = power; }
    public void setstrafePower(double power){ strafePower = power; }


    //premade directions
    public void forward(double magnitude) {
        leftPower = magnitude;
        rightPower = magnitude;
        strafePower = 0;
    }
    public void back(double magnitude) {
        leftPower = -magnitude;
        rightPower = -magnitude;
        strafePower = 0;
    }
    public void left(double magnitude) {
        leftPower = 0;
        rightPower = 0;
        strafePower = magnitude;
    }
    public void right(double magnitude) {
        leftPower = 0;
        rightPower = 0;
        strafePower = -magnitude;
    }







    //THE ADDERS ##################################################
    //MAKE SURE NOT TO RUN MULTIPLE TIMES OR IN A LOOP
    public void addPowers(double leftPower, double rightPower, double strafePower, double dPower) {
        this.leftPower = this.leftPower + (leftPower);
        this.rightPower = this.rightPower + (rightPower);
        this.strafePower = this.strafePower + (strafePower);
    }
    public void addleftPower(double add){ leftPower = leftPower + add; }
    public void addrightPower(double add){ rightPower = rightPower + add; }
    public void addstrafePower(double add){ strafePower = strafePower + add; }












}//end of class
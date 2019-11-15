/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;



@TeleOp(name="TesterIterative", group ="concept")

public class TesterIterative extends OpMode {

    Hardware hardware;


    String[] titles = new String[] {"p", "i", "d", "b", "f", "t", "vt"}; //names of the tuner values
    double[] values = new double[] {0.002, 6.77e-5, -0.0064, 0.229, 0.10, 5, 0.1}; //default tuner values
    Tuner tuner;



    public void init(){
        hardware = new Hardware(hardwareMap, telemetry, true);
        tuner = new Tuner(titles, values, gamepad1, telemetry);
        tuner.incrementSpeed = 0.001;
    }

    boolean running = false;

    Calculate.PIDF visionPID = new Calculate.PIDF();

    public void loop() {
        if(gamepad1.x){
            running = true;
        }else if(gamepad1.y){
            running = false;
        }

        tuner.tune();
        visionPID.setConstants(tuner.get("p"),
                tuner.get("i"),
                tuner.get("d"),
                tuner.get("b"),
                tuner.get("f"),
                tuner.get("t"),
                tuner.get("vt"));

        if(running && hardware.vuforiaPhone.getSkystoneTranslation() != null){
            telemetry.addData("skystone", hardware.vuforiaPhone.getSkystoneTranslation().get(1));

            double power = visionPID.loop(hardware.vuforiaPhone.getSkystoneTranslation().get(1), 0);
            hardware.steadyTranslationPIDF(power, 0);

            if(visionPID.inTolerance() && visionPID.inVelTolerance()){
                running = false;
            }

        }else{
            hardware.drivetrain.setPowers(0,0,0);
        }

        hardware.drivetrain.execute();
        telemetry.addData("running", running);
        telemetry.addData("P", visionPID.getPID()[0]);
        telemetry.addData("I", visionPID.getPID()[1]);
        telemetry.addData("D", visionPID.getPID()[2]);
        telemetry.addData("F", visionPID.getPID()[3]);
        telemetry.addData("vel", visionPID.getVel());
        telemetry.update();

    }

}

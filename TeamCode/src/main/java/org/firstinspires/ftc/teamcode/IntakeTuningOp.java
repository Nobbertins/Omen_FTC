/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

/*
Yellow to Gray (Servo signal pin)

CONTROLLER LAYOUT:

Two joysticks - driving

LT - linear slide up
RT - linear slide down
LB - run intake
RB - run intake backwards
Y - toggle slide servos
B - toggle intake drop servos
A - toggle deposit servo
A and X - launch plane

CONFIG MOTOR NAMES:

*REMEMBER THAT THE DEADWHEEL ENCODERS ARE ALSO WIRED TO THESE MOTORS

Control Hub:
Motors (port, name):
0 - bl
1 - fl
2- rraise
3 - intake (lateral deadwheel)
Servos:
0 - ldrop
1 -
2 -
3 -
4 -
5 -
Expansion Hub:
Motors:
0 - br
1 - fr
2 - lraise
3 - hang (axial deadwheel)
Servos:
0 - rdrop
1 - launch
2 - lslide
3 - rslide
4 -
5 - deposit
 */

//define OP
@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled

//define OP class
public class IntakeTuningOp extends LinearOpMode {

    // declare time counter variable
    private ElapsedTime runtime = new ElapsedTime();

    //declare al DCMotors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor rraiseMotor = null;

    private DcMotor lraiseMotor = null;

    private DcMotor intakeMotor = null;
    private DcMotor hangMotor = null;
    //declare all servo motors
    private Servo lslideServo = null;

    private Servo rslideServo = null;

    private Servo ldropServo = null;

    private Servo rdropServo = null;

    private Servo depositServo = null;
    private Servo launchServo = null;
    //private CRServo contServo = null;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        rraiseMotor = hardwareMap.get(DcMotor.class, "rraise");
        lraiseMotor = hardwareMap.get(DcMotor.class, "lraise");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        hangMotor = hardwareMap.get(DcMotor.class, "hang");
        lslideServo = hardwareMap.get(Servo.class, "lslide");
        rslideServo = hardwareMap.get(Servo.class, "rslide");
        ldropServo = hardwareMap.get(Servo.class, "ldrop");
        rdropServo = hardwareMap.get(Servo.class, "rdrop");
        depositServo = hardwareMap.get(Servo.class, "deposit");
        launchServo = hardwareMap.get(Servo.class, "launch");
        //contServo = hardwareMap.crservo.get("contservo");

        //set all default directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        hangMotor.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //initialize motor states
        boolean dropState = false;
        boolean bPressed = false;

        double dropServoPosition = 0;
        boolean intakeMotorDirection = true;
        boolean lbPressed = false;

        boolean runIntakeMotor = false;
        boolean rbPressed = false;

        ldropServo.setPosition(0);
        rdropServo.setPosition(0.5);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.left_trigger > 0){
                dropServoPosition += 0.0001;
            }
            if(gamepad1.right_trigger > 0){
                dropServoPosition -= 0.0001;
            }
            ldropServo.setPosition(dropServoPosition);
            rdropServo.setPosition(dropServoPosition + 0.5);
            //switch drop servos position on b press
            if(gamepad1.b && !bPressed) {
                dropState = !dropState;
                if (dropState) {
                    ldropServo.setPosition(0.5);
                    rdropServo.setPosition(0);
                } else {
                    ldropServo.setPosition(0);
                    rdropServo.setPosition(0.5);
                }
            }
            bPressed = gamepad1.b;

            //switch intake motor direction on LB
            if(gamepad1.left_bumper && !lbPressed) {
                intakeMotorDirection = !intakeMotorDirection;
                if(intakeMotorDirection){
                    intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                }
                else{
                    intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                }
                //run motors with new direction if already running
                if(runIntakeMotor){
                    intakeMotor.setPower(0.9);
                }
                else{
                    intakeMotor.setPower(0);
                    intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                }
            }
            //toggle intake
            lbPressed = gamepad1.left_bumper;
            //toggle running intake motor on RB
            if(gamepad1.right_bumper && !rbPressed) {
                runIntakeMotor = !runIntakeMotor;
                if(runIntakeMotor){
                    //default direction forward
                    intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                    intakeMotor.setPower(0.9);
                }
                else{
                    intakeMotor.setPower(0);
                }
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Drop Left/Right Position", "%4.2f, %4.2f", ldropServo.getPosition(), rdropServo.getPosition());
            telemetry.update();
        }
    }}

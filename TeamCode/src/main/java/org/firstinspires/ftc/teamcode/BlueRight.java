/* Copyright (c) 2017 FIRST. All rights reserved.
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

import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="BlueRight", group = "Blue")
// @Disabled
public class BlueRight extends LinearOpMode {

    // Declare OpMode members.
    //dadad
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private DcMotorEx LF = null;
    private DcMotorEx LB = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private DcMotorEx carousel = null;
    private DcMotorEx viper = null;
    private Camera Camera = null;

    private final double encoderConstant = 45.2847909695;

    private final double carouselConstant = 45.8366237;

    private final double viperConstant = 40.151561;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Stat", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF  = hardwareMap.get(DcMotorEx.class, "left_front");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");
        carousel = hardwareMap.get(DcMotorEx.class, "sustainable");
        viper = hardwareMap.get(DcMotorEx.class, "Sussy Wussy");
        Camera = hardwareMap.get(Camera.class, "amogOS");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);
        RF.setDirection(DcMotorEx.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            telemetry.addData("Running: ", "Blue Right");
            telemetry.update();
            DuckDetector();

            driveForward(18, 100);
            Thread.sleep(100);

            turnLeft(3, 200);
            Thread.sleep(100);

            viperExtend(10);

            turnRight(10, 100);
            Thread.sleep(100);

            driveForward(27, 200);
            Thread.sleep(100);

            spinCarousel(6, 100);
            Thread.sleep(100);

            driveBack(3, 100);
            Thread.sleep(100);

            turnRight(10, 100);
            Thread.sleep(10);

            driveForward(90, 200);
            Thread.sleep(100);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void quitDriving() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }

    public void spinCarousel(double distance, double speed) {
        carousel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setTargetPosition((int) (distance * carouselConstant));
        carousel.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        carousel.setVelocity(speed);

        while (carousel.isBusy()) {
            idle();
        }

        carousel.setPower(0);
    }

    public void spinCarouselBackwards(double distance, double speed) {
        carousel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setTargetPosition((int) (-1 * (distance * carouselConstant)));
        carousel.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        carousel.setVelocity(speed);

        while (carousel.isBusy()) {
            idle();
        }

        carousel.setPower(0);
    }

    public void driveForward(double distance, double speed) {
        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LF.setTargetPosition((int) (distance * encoderConstant));
        LB.setTargetPosition((int) (distance * encoderConstant));
        RF.setTargetPosition((int) (distance * encoderConstant));
        RB.setTargetPosition((int) (distance * encoderConstant));

        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LF.setVelocity(speed);
        LB.setVelocity(speed);
        RF.setVelocity(speed);
        RB.setVelocity(speed);

        while (LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy()) {
            idle();
        }

        quitDriving();

    }

    public void driveBack(double distance, double speed) {
        LF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        LB.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        RF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        RB.setTargetPosition((int) (-1 * (distance * encoderConstant)));

        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LF.setVelocity(speed);
        LB.setVelocity(speed);
        RF.setVelocity(speed);
        RB.setVelocity(speed);

        while (LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy()) {
            idle();
        }

        quitDriving();
    }

    public void turnRight(double distance, double speed) {
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        RF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        RB.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        LF.setTargetPosition((int) (distance * encoderConstant));
        LB.setTargetPosition((int) (distance * encoderConstant));

        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        RF.setVelocity(-1 * speed);
        RB.setVelocity(-1 * speed);
        LF.setVelocity(speed);
        LB.setVelocity(speed);

        while (RF.isBusy() || RB.isBusy() || LF.isBusy() || LB.isBusy()) {
            idle();
        }

        quitDriving();
    }

    public void turnLeft(double distance, double speed) {
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        RF.setTargetPosition((int) (distance * encoderConstant));
        RB.setTargetPosition((int) (distance * encoderConstant));
        LF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        LB.setTargetPosition((int) (-1 * (distance * encoderConstant)));

        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        RF.setVelocity(speed);
        RB.setVelocity(speed);
        LF.setVelocity(-1 * speed);
        LB.setVelocity(-1 * speed);

        while (RF.isBusy() || RB.isBusy() || LF.isBusy() || LB.isBusy()) {
            idle();
        }

        quitDriving();
    }



    public void carouselSpin(double time) throws InterruptedException {
        carousel.setVelocity(0.25);
        Thread.sleep((long) (time * 1000));
        carousel.setVelocity(0);
    }

    public void viperExtend(double distance) throws InterruptedException {
        viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper.setTargetPosition((int) (distance * viperConstant));
        viper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper.setVelocity(1);

        while (viper.isBusy()) {
            idle();
        }

        viper.setPower(0);
    }
    public static class DuckDetector extends OpenCvPipeline {
        Telemetry telemetry;
        Mat mat = new Mat();
        public enum Location {
            LEFT,
            RIGHT,
            MIDDLE,
            NOT_FOUND
        }
        private org.firstinspires.ftc.teamcode.DuckDetector.Location location;

        final Rect LEFT_ROI = new Rect(
                new Point(90, 75),
                new Point(150, 115));
        final Rect MIDDLE_ROI = new Rect(
                new Point(160,75),
                new Point(220,115)
        );
        final Rect RIGHT_ROI = new Rect(
                new Point(230, 75),
                new Point(290, 115));
        double PERCENT_COLOR_THRESHOLD = 0.03;

        public DuckDetector(Telemetry t) { telemetry = t; }

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(140, 50, 69);
            Scalar highHSV = new Scalar(160, 255, 255);

            Core.inRange(mat, lowHSV, highHSV, mat);

            Mat left = mat.submat(LEFT_ROI);
            Mat right = mat.submat(RIGHT_ROI);
            Mat middle = mat.submat(MIDDLE_ROI);

            double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
            double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

            left.release();
            right.release();

            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
            telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

            boolean isLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean isRight = rightValue > PERCENT_COLOR_THRESHOLD;
            boolean isMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

            if (!isLeft && !isRight && !isMiddle) {
                location = org.firstinspires.ftc.teamcode.DuckDetector.Location.NOT_FOUND;
                telemetry.addData("Duck Location", "not found");
            }
            else if (isLeft){
                location = org.firstinspires.ftc.teamcode.DuckDetector.Location.LEFT;
                telemetry.addData("Duck Location", "left");
            }
            else if (isMiddle) {
                location = org.firstinspires.ftc.teamcode.DuckDetector.Location.MIDDLE;
                telemetry.addData("Duck Location", "middle");
            }
            else {
                location = org.firstinspires.ftc.teamcode.DuckDetector.Location.RIGHT;
                telemetry.addData("Duck Location", "right");
            }
            telemetry.update();

            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

            Scalar colorStone = new Scalar(255, 0, 0);
            Scalar colorSkystone = new Scalar(0, 255, 0);

            Imgproc.rectangle(mat, LEFT_ROI, location == org.firstinspires.ftc.teamcode.DuckDetector.Location.LEFT? colorSkystone:colorStone);
            Imgproc.rectangle(mat, RIGHT_ROI, location == org.firstinspires.ftc.teamcode.DuckDetector.Location.RIGHT? colorSkystone:colorStone);
            Imgproc.rectangle(mat, MIDDLE_ROI, location == org.firstinspires.ftc.teamcode.DuckDetector.Location.MIDDLE? colorSkystone:colorStone);

            return mat;
        }

        public org.firstinspires.ftc.teamcode.DuckDetector.Location getLocation() {
            return location;
        }
    }

}

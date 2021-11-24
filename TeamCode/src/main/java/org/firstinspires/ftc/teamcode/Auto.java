package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Autonomous", group = "Autonomous")
public class Auto extends LinearOpMode {

    private DcMotorEx LF = null;
    private DcMotorEx LB = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private OpenCvCamera phoneCam;
    private DcMotorEx carousel = null;

    private final double encoderConstant = 45.2847909695;

    @Override
    public void runOpMode() throws InterruptedException{
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");
        carousel = hardwareMap.get(DcMotorEx.class, "sustainable");

        LF.setDirection(DcMotorEx.Direction.FORWARD);
        LB.setDirection(DcMotorEx.Direction.FORWARD);
        // reverse right side
        RF.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.REVERSE);

        carousel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        DuckDetector detector = new DuckDetector(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();
        switch (detector.getLocation()) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }
        phoneCam.stopStreaming();


        while (opModeIsActive()) {

        }
    }


    public void quitDriving() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);

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


}


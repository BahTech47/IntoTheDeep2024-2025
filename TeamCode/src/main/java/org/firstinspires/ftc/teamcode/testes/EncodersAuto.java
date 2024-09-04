package org.firstinspires.ftc.teamcode.testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="EncodersAuto", group="BahTech")
public class EncodersAuto extends LinearOpMode {

    private static final double TicksPorRotação = 28;
    private static final double redução = 5;
    private static final double circunferência = 94*Math.PI;
    private static final double TPM = TicksPorRotação*redução/circunferência;
    private final double kp = 0.001;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    @Override
    public void runOpMode() throws InterruptedException {

        int aPosition = - 10;

        int bPosition = 0;

        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BR = hardwareMap.dcMotor.get("BR");

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setTargetPosition(bPosition);
        BL.setTargetPosition(bPosition);
        FR.setTargetPosition(bPosition);
        BR.setTargetPosition(bPosition);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            FL.setTargetPosition(aPosition);
            BL.setTargetPosition(aPosition);
            FR.setTargetPosition(aPosition);
            BR.setTargetPosition(aPosition);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            smoother(FL,0.15);
            smoother(BL,0.15);
            smoother(FR,0.15);
            smoother(BR,0.15);

            telemetry.addData("posição", FL.getCurrentPosition());
            telemetry.update();
        }

    }

    private void smoother(DcMotor motor, Double targetVelocity){
        int targetPos = motor.getTargetPosition();
        int currPos = motor.getCurrentPosition();

        double power = Math.abs(targetPos - currPos) * kp;
        if (power <= targetVelocity)
            motor.setPower(power);
        else
            motor.setPower(targetVelocity);

    }

}
package org.firstinspires.ftc.teamcode.testes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="TesteAuto", group="BahTech")
public class TesteAuto extends LinearOpMode {

    private final double kp = 0.001;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private int teste = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        int a = - 288;
        int b = - 672;
        int c = 48;
        int d = -624;
        int e = 576;
        int f = 288;
        int g = 624;

        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BR = hardwareMap.dcMotor.get("BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            FL.setTargetPosition(f);
            BL.setTargetPosition(a);
            FR.setTargetPosition(a);
            BR.setTargetPosition(f);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            smoother(FL,0.15);
            smoother(BL,0.15);
            smoother(FR,0.15);
            smoother(BR,0.15);
            sleep(1200);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setTargetPosition(b);
            BL.setTargetPosition(b);
            FR.setTargetPosition(b);
            BR.setTargetPosition(b);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            smoother(FL,0.15);
            smoother(BL,0.15);
            smoother(FR,0.15);
            smoother(BR,0.15);
            sleep(1200);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setTargetPosition(c);
            BL.setTargetPosition(c);
            FR.setTargetPosition(c);
            BR.setTargetPosition(c);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            smoother(FL,0.15);
            smoother(BL,0.15);
            smoother(FR,0.15);
            smoother(BR,0.15);
            sleep(1200);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setTargetPosition(d);
            BL.setTargetPosition(g);
            FR.setTargetPosition(g);
            BR.setTargetPosition(d);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            smoother(FL,0.15);
            smoother(BL,0.15);
            smoother(FR,0.15);
            smoother(BR,0.15);
            sleep(1200);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setTargetPosition(e);
            BL.setTargetPosition(e);
            FR.setTargetPosition(e);
            BR.setTargetPosition(e);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            smoother(FL,0.15);
            smoother(BL,0.15);
            smoother(FR,0.15);
            smoother(BR,0.15);
            sleep(1200);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1200);


            telemetry.addData("posição", FL.getCurrentPosition());
            telemetry.addData("teste", teste);
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

        if (power <= 0.003)
            motor.setPower(0);

    }

}
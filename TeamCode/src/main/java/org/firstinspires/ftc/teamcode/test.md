        leftServo.setPosition(leftServoPosition);
        rightServo.setPosition(rightServoPosition);

        leftMotor.setTargetPosition(14000);
        rightMotor.setTargetPosition(14000);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(Math.abs(.1));
        rightMotor.setPower(Math.abs(.1));
        //Setting x to value to 54 to practice pushing code to git
        x=54;
        while (opModeIsActive() &&
                (leftMotor.isBusy() && rightMotor.isBusy())) {
            telemetry.addData("Path1", "leftMotor: %d", leftMotor.getCurrentPosition());
            telemetry.addData("Path2", "rightMotor: %d", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);






//1440
import adafruithat.AdafruitServo;

public class OldCode {
	
/*	
	 * Because the Adafruit servo HAT uses PWMs that pulse independently of
	 * the Raspberry Pi the servos will continue to drain power
	 * if the program abnormally terminates. 
	 * A shutdown hook like the one in this example is useful to stop the 
	 * servos when the program is abnormally interrupted.
	 			AdafruitServo servo  = servoHat.getServo("S01");	
	Runtime.getRuntime().addShutdownHook(new Thread() {
	    public void run() { 
	    	System.out.println("Turn off all servos");
	    	servoHat.stopAll();		    	
	    }
	 });
			
	//Set pulse width operating limits of servo, consult servo data sheet
	//for manufaturer's recommended operating limits. Values
	//are in milliseconds;
	float minimumPulseWidth = 0.6f;
	float neutralPulseWidth = 1.5f;
	float maximumPulseWidth = 2.4f;
	servo.setOperatingLimits(minimumPulseWidth, neutralPulseWidth, maximumPulseWidth);
	
	//Set relative range of servo for setPosition() commanding
	//(The default is 0.0 to 1.0)
	servo.setPositionRange(0.0F, 100.0f);

	//Option 1: Command servo position by using pulse width
	for (float pulseWidth: new float[] {minimumPulseWidth, neutralPulseWidth, maximumPulseWidth}) {
		servo.setPulseWidth(pulseWidth);
		servoHat.sleep(1000);
	}		

	//Move servo to neutral position.
	servo.setPulseWidth(neutralPulseWidth);
	servoHat.sleep(1000);
	
	//Option 2: Command servo position by relative values			
	for (float position: new float[] {0.0f, 10.0f, 20.0f, 30.0f, 040.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.00f, 0.0f, 100.0f, 0.0f, 100.0f}) {
		System.out.format("Move to position: %8.1f\n", position);
		servo.setPosition(position);
		servoHat.sleep(1000);
	}
	*/

}

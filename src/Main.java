import kinematics.Robot;
import util.Trig;

public class Main{
	
	//static Joystick joystick;

	public static void main(String[] args) {

		
	/*	AdafruitServoHat servoHat;
		int servoHATAddress = 0X40;
		AdafruitServo servo1;

		servoHat = new AdafruitServoHat(servoHATAddress);
		servo1 = servoHat.getServo("S16");
		servo1.setOperatingLimits(0.6f, 1.55f, 2.55f);
		servo1.setPositionRange(0.0f, 180.0f);
		servo1.setPosition(90.0f);
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		servo1.setPosition(0.0f);
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		servo1.setPosition(90.0f);*/
		
		//joystick = new Joystick();
		//mathTest(0,0,-5.0, true);
		//mathTest(0,0,-5.0, false);
		Robot robot = new Robot();
		robot.start();
		//System.out.println(Body.getLocalCornerPos(new Position(0,0,0,0,32,10))[0].x);
		
		/*for(int i = -13; i < 14; i++){
			mathTest((double)i,0.0,-22.0,false);
		}*/
		
		//for(int i = 0; i < 1000; i++){
			//KeyManager.tick();
			//if(KeyManager.w)System.out.print("s");
			//joystick.updateJoystick();
		//}
	
	}	
	
/*
	public static final double FEMUR = 1.6;
	public static final double TIBIA = 2.3585;
	public static final double TARSUS = 3.4;
	public static final double SERVOWIDTH = 1.927;*/
	
	public static final double FEMUR = 1.8;
	public static final double TIBIA = 12.0;
	public static final double TARSUS = 14.0;
	public static final double SERVOWIDTH = .25;
	
	public static void mathTest(double x, double y, double z, boolean left){
		
		double hipAngle = 90;
		double kneeAngle = 90;
		double ankleAngle = 90;
		/*float hipCenter = 90;
		float kneeCenter = 90;
		float ankleCenter = 7;*/
		float hipCenter = 0;
		float kneeCenter = 0;
		float ankleCenter = 0;
		
		if(!left) y = -y;
		
		double C = Math.sqrt(z*z + y*y);
		
		double c = 90 + Trig.atan(FEMUR/SERVOWIDTH*2);//Okay
		double A = Math.sqrt(FEMUR*FEMUR + SERVOWIDTH*SERVOWIDTH/4);
		if((left && y > 0) || (!left && y > 0)) hipAngle = hipCenter - (180 - c - Trig.asin(A*Trig.sin(c)/C) + Trig.acos(-z/C) - Trig.atan(SERVOWIDTH/FEMUR/2));
		else hipAngle = hipCenter - (180 - c - Trig.asin(A*Trig.sin(c)/C) - Trig.acos(-z/C) - Trig.atan(SERVOWIDTH/FEMUR/2));
		if(!left){
			if(y > 0) hipAngle = hipCenter - (hipAngle - hipCenter);
			else hipAngle =  hipCenter + (hipCenter - hipAngle);
		}
		double B = C * Trig.sin(180 - c - Trig.asin(A*Trig.sin(c)/C))/Trig.sin(c);
		double L = Math.sqrt(B*B + x*x);
		double absoluteKneeAngle = Trig.acos((L*L + TIBIA*TIBIA - TARSUS*TARSUS)/(2*L*TIBIA));
		kneeAngle = kneeCenter - (absoluteKneeAngle - Trig.atan2(x, B));
		if(left) ankleAngle = ankleCenter + (180 - Trig.acos((TIBIA*TIBIA + TARSUS*TARSUS - L*L)/(2*TIBIA*TARSUS)));
		else ankleAngle = 180 - (ankleCenter + (180 - Trig.acos((TIBIA*TIBIA + TARSUS*TARSUS - L*L)/(2*TIBIA*TARSUS))));
		
		
		double ka = 20.0;//lb/in
		double N = 25.0;//lb
		double la = 3.0;//in (unstretched spring length)
		double cx = 1.5;
		double cy = 7.0;//location of end of spring
		double r = 2.0;//location of lever arm
		double ABx = r*Math.sin(Math.PI - Math.toRadians(ankleAngle));
		double ABy = r*Math.cos(Math.PI - Math.toRadians(ankleAngle));
		double BC = Math.sqrt((cx-ABx)*(cx-ABx) + (cy-ABy)*(cy-ABy));
		
		double Ms = (ABx*(cy-ABy)-(cx-ABx)*ABy)*ka*(1.0-la/BC);
		double Ma = N*TARSUS*Math.sin(Math.PI - Math.toRadians(ankleAngle) + Math.toRadians(kneeAngle));
		double Mma = Ma-Ms;
		
		//kneeAngle = 36.9;
		double kk = 20.0;//lb/in
		double lk = 3.0;//in (unstretched spring length)
		double dx = 3.0;
		double dy = 4.0;//location of end of spring
		double gx = 3.5;
		double gy = .5;//location of end of spring
		double ADx = -dx*Math.cos(Math.toRadians(-kneeAngle)) + dy*Math.sin(Math.toRadians(-kneeAngle));
		double ADy = -dx*Math.sin(Math.toRadians(-kneeAngle)) - dy*Math.cos(Math.toRadians(-kneeAngle));
		double DG = Math.sqrt((-gx-ADx)*(-gx-ADx) + (gy-ADy)*(gy-ADy));
		
		
		//double Msk = kk*(1-lk/DG)*(ADx*(-gy - ADy) - ADy*(gx - ADx));
		//double Msk = kk*(1-lk/DG)*(-gx*(gy - ADy) - gy*(-ADx - gx));
		double Msk = kk*(1-lk/DG)*(-gx*(gy - ADy) - gy*(-ADx - gx));
		double Mk = N*x;
		double Mmk = Mk - Msk;
		
		
		//System.out.println(BC);
		//System.out.println(Ma + " lb-in");
		//System.out.println();
		System.out.println(x +", " + Mma + ", " + Ma +", " + Ms + ", " + (12*(Mma*0.112985/.71)/16.0*18.0/56.0) + ", " + Mmk + ", " + Mk + ", " + Msk);
		//System.out.println(hipAngle);
		//System.out.println(kneeAngle);
		//System.out.println(ankleAngle);
		
	}
}
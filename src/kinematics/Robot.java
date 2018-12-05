package kinematics;

import java.awt.Graphics;
import java.awt.image.BufferStrategy;

import adafruithat.AdafruitServoHat;
import input.KeyManager;
import motion.RobotMotion;
import simulationdisplay.Display;
import util.Constants;
import util.Position;

public class Robot extends Thread{
	
	private Display display;
	public String title = "Quadruped Simulation";	
	private BufferStrategy bs;
	private Graphics g;
	private KeyManager keyManager;
	
	AdafruitServoHat servoHat;
	final int servoHATAddress = 0X40;
	private boolean running = true;

	Leg frontLeftLeg, frontRightLeg, hindLeftLeg, hindRightLeg;
	public Leg legs[];
	//public Position legPos[] = {new Position(3.0,0,-6.0,0,0,0), new Position(0,0,0,0,0,0), new Position(1.927/2,-3.0,-6.0,0,0,0), new Position(0,0,0,0,0,0)};
	public Position GlobalRobotPos = new Position(0,0,0,0,0,0);
	private RobotMotion motion;
	
	public Robot(){
		if(!Constants.SIMULATION) {
			servoHat = new AdafruitServoHat(servoHATAddress);
			frontLeftLeg = new Leg(servoHat.getServo("S13"), servoHat.getServo("S14"), servoHat.getServo("S15"), true);
			frontRightLeg = new Leg(servoHat.getServo("S09"), servoHat.getServo("S10"), servoHat.getServo("S11"), false);
			hindLeftLeg = new Leg(servoHat.getServo("S05"), servoHat.getServo("S06"), servoHat.getServo("S07"), true);
			hindRightLeg = new Leg(servoHat.getServo("S01"), servoHat.getServo("S02"), servoHat.getServo("S03"), false);
		}
		else{
			frontLeftLeg = new Leg(true);
			frontRightLeg = new Leg(false);
			hindLeftLeg = new Leg(true);
			hindRightLeg = new Leg(false);
		}
//		legs[0] = frontLeftLeg;
//		legs[1] = frontRightLeg;
//		legs[2] = hindLeftLeg;
//		legs[3] = hindRightLeg;
		motion = new RobotMotion(frontLeftLeg, frontRightLeg, hindLeftLeg, hindRightLeg);
		keyManager = new KeyManager();
	}
	
	public void setStartPosition(){
		
	}
	
	public void init(){
		setStartPosition();
		if(Constants.SIMULATION) display = new Display(title, Constants.SIMWIDTH, Constants.SIMHEIGHT);
		else display = new Display(title, 100, 100);
		display.getFrame().addKeyListener(keyManager);
	}
	
	public void stopRobot(){
		servoHat.stopAll();//stop all servos
	}
	
	public void update(){
		motion.update();
	}
	
	public void render(){
		bs = display.getCanvas().getBufferStrategy();
		if(bs == null){
			display.getCanvas().createBufferStrategy(3);
			return;
		}
		g = bs.getDrawGraphics();
		//Clear Screen
		g.clearRect(0, 0, Constants.SIMWIDTH, Constants.SIMHEIGHT);

		//Draw Coordinate Axes
		g.drawLine(0, Constants.y0, Constants.SIMWIDTH, Constants.y0);
		g.drawLine(Constants.x0, 0, Constants.x0, Constants.SIMHEIGHT);
		motion.render(g);
		
		bs.show();
		g.dispose();
	}
	
	public Position calcNewRobotPos(){
		return new Position(0,0,0,0,0,0);
	}
	

	@Override
	public void run() {
			init();
			
			double fps = Constants.UPDATESPERSECOND;// updates per second
			double timePerTick = 1000000000/fps;
			double delta = 0;
			long now;
			long lastTime = System.nanoTime();
			long timer = 0;
			int ticks = 0;
			long totalSeconds = 0;
			
			while(running){
				now = System.nanoTime();
				delta += (now - lastTime) / timePerTick;
				timer += now - lastTime;
				lastTime = now;
				
				if(delta >= 1){
					update();
					if(Constants.SIMULATION) render();
					ticks++;
					delta--;
					if(motion.end) running = false;
				}
				
				if(timer >= 1000000000){
					//System.out.println("Ticks and Frames " + ticks);
					timer = 0;
					ticks = 0;
					totalSeconds++;
				}
			}
			
			System.out.println("Done!!!");
			
			stopRobot();
	}
	
	

}

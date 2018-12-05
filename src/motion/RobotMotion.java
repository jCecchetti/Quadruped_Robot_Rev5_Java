package motion;

import java.awt.Color;
import java.awt.Graphics;

import com.studiohartman.jamepad.ControllerManager;
import com.studiohartman.jamepad.ControllerState;

import input.Joystick;
import input.KeyManager;
import kinematics.Body;
import kinematics.Leg;
import util.Constants;
import util.Position;
import util.Timer;
import util.Trig;

public class RobotMotion {
	
	public Position globalRobotPos = new Position(0,0,6.5,0,0,0);
	public Position localRobotPos = new Position(0,0,0,0,0,0);
	public Position[] globalFeetPos = {new Position(Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(-Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(-Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0,0,0,0)};
	public Position[] globalCornerPos = {new Position(Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(-Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(-Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0,0,0,0)};
	public Position[] globalStepCenter = {new Position(Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(-Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(-Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0,0,0,0)};

	public Position[] lastGlobalStepCenter = {new Position(Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(-Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0,0,0,0),
			new Position(-Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0,0,0,0)};

	private double robotSpeed = 1.0;// in/s
	private double currentRobotSpeedX = 0;
	private double currentRobotSpeedY = 0;
	private double turningSpeed = 7.0;// degrees/s
	private double updateRate = Constants.UPDATESPERSECOND;
	private Leg frontLeftLeg, frontRightLeg, hindLeftLeg, hindRightLeg;
	//private Leg[] legs;
	private double stepTime = 1.0;
	private double stepLengthX = 0;
	private double stepLengthY = 0;
	private double stepHeight = 1;
	private double deadBand = .05;
	private Timer timer = new Timer();
	public boolean end = false;
	//private Joystick j = new Joystick();
	//ControllerManager controller;
	//ControllerState j;
	
	public RobotMotion(Leg frontLeftLeg, Leg frontRightLeg, Leg hindLeftLeg, Leg hindRightLeg){
		//this.legs = legs;
		this.frontLeftLeg = frontLeftLeg;
		this.frontRightLeg = frontRightLeg;
		this.hindLeftLeg = hindLeftLeg;
		this.hindRightLeg = hindRightLeg;
		//controller = new ControllerManager();
		//controller.quitSDLGamepad();
		//controller.initSDLGamepad();
	}

	public void updateGlobalRobotPos(){
		KeyManager.tick();
		if(KeyManager.numup){
			globalRobotPos.x += robotSpeed/updateRate;
			currentRobotSpeedX = robotSpeed; 
		}
		else if(KeyManager.up){
			globalRobotPos.x += 2.0*robotSpeed/updateRate;
			currentRobotSpeedX = 2.0*robotSpeed;
		}
		else if(KeyManager.numdown) {
			globalRobotPos.x -= robotSpeed/updateRate;
			currentRobotSpeedX = -robotSpeed; 
		}
		else if(KeyManager.down){
			globalRobotPos.x -= 2.0*robotSpeed/updateRate;
			currentRobotSpeedX = -2.0*robotSpeed;
		}
		else currentRobotSpeedX = 0; 
		if(KeyManager.numleft || KeyManager.q){ 
			globalRobotPos.y += robotSpeed/updateRate;
			currentRobotSpeedY = robotSpeed;
		}
		else if(KeyManager.numright || KeyManager.e){
			globalRobotPos.y -= robotSpeed/updateRate;
			currentRobotSpeedY = -robotSpeed;
		}
		else currentRobotSpeedY = 0;

		//System.out.println(globalRobotPos.x);
		if(KeyManager.j) localRobotPos.yaw += turningSpeed/updateRate;
		if(KeyManager.l) localRobotPos.yaw -= turningSpeed/updateRate;

		if(KeyManager.a) localRobotPos.y += 2.5/updateRate;
		else if(KeyManager.d) localRobotPos.y -= 2.5/updateRate;
		if(KeyManager.s){
			localRobotPos.x = 0;
			localRobotPos.y = 0;
			localRobotPos.z = 0;
			localRobotPos.roll = 0;
			localRobotPos.yaw = 0;
		}
		if(KeyManager.w) localRobotPos.x += 2.5/updateRate;
		else if(KeyManager.x) localRobotPos.x -= 2.5/updateRate;
		if(KeyManager.e) localRobotPos.z += 1.5/updateRate;
		else if(KeyManager.c) localRobotPos.z -= 1.5/updateRate;
		if(KeyManager.f) localRobotPos.yaw += 25.0/updateRate;
		else if(KeyManager.h) localRobotPos.yaw -= 25.0/updateRate;
		if(KeyManager.r) localRobotPos.roll += 25.0/updateRate;
		else if(KeyManager.y) localRobotPos.roll -= 25.0/updateRate;
		if(KeyManager.v) localRobotPos.pitch += 20.0/updateRate;
		else if(KeyManager.n) localRobotPos.pitch -= 20.0/updateRate;

		/*j = controller.getState(0);
		if(!j.isConnected) System.out.println("Controller Not Connected!");
		if(j.a) System.out.println("a");
		if(currentState == State.walking) {
			globalRobotPos.x += j.rightStickY*robotSpeed/updateRate;
			globalRobotPos.y += j.rightStickX*robotSpeed/updateRate;
			currentRobotSpeedX = j.rightStickY*robotSpeed;
			currentRobotSpeedY = j.rightStickX*robotSpeed;
			localRobotPos.yaw += j.leftStickX*turningSpeed/updateRate;
			
		}
		else if(currentState == State.standing){
			localRobotPos.x += j.rightStickY*2.5/updateRate;
			localRobotPos.y += j.rightStickX*2.5/updateRate;
			localRobotPos.yaw += j.leftStickX*25.0/updateRate;
			if(j.lb) localRobotPos.roll += 25.0/updateRate;
			else if(j.rb) localRobotPos.roll -= 25.0/updateRate;
			localRobotPos.pitch += j.leftStickY*20.0/updateRate;
		}
		else if(currentState == State.stopped){
			
		}
		
		localRobotPos.z += j.rightTrigger*1.5/updateRate;
		localRobotPos.z -= j.leftTrigger*1.5/updateRate;
		*/
	}
	
	private enum leg{frontLeft (0), frontRight (1), rearLeft(2), rearRight (3);
		
		private int legNum;
		leg(int legNum) {
			this.legNum = legNum;
		}
		
		int getLegNum() {
			return legNum;
		}
	}
	public enum State{walking, trotting, laying, stopped, stopping, standing}
	
	public State currentState = State.standing;
	
	public void setWantedState(State wantedState){
		
		switch(wantedState){
		case walking:
			handleWalkingLegs();
			break;
		case trotting:
			handleTrottingLegs();
			break;
		case laying:
			break;
		case stopping:
		case stopped:
			home();
			break;
		case standing:
			handleStanding();
			break;
		}
	}
	
	public void update(){
		KeyManager.tick();
		if(KeyManager.num5) currentState = State.stopped;
		else if(KeyManager.i) currentState = State.trotting;
		else if(KeyManager.numup || KeyManager.numdown || KeyManager.numleft || KeyManager.numright) currentState = State.walking;
		//else if(KeyManager.j || KeyManager.k || KeyManager.l || KeyManager.i || KeyManager.o || KeyManager.u) currentState = State.standing;
		else if(KeyManager.space) end = true;
		else if(KeyManager.s) currentState = State.standing;
		setWantedState(currentState);
		
		/*j = controller.getState(0);
		if(j.y) currentState = State.stopped;
		else if(j.b) currentState = State.trotting;
		else if(j.a) currentState = State.walking;
		//else if(KeyManager.j || KeyManager.k || KeyManager.l || KeyManager.i || KeyManager.o || KeyManager.u) currentState = State.standing;
		else if(j.back) end = true;
		else if(j.x) currentState = State.standing;
		setWantedState(currentState);
		*/
	}
	
	private leg steppingLeg = leg.frontLeft;
	private leg lastSteppingLeg = leg.frontLeft;
	private Step step = new Step(0);
	private Step stepMir = new Step(2);
	
	public void handleWalkingLegs(){
		updateGlobalRobotPos();
		globalStepCenter = Body.getGlobalStepCenter(localRobotPos, globalRobotPos);
		globalCornerPos = Body.getGlobalCornerPos(localRobotPos, globalRobotPos);
		frontLeftLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[0], globalFeetPos[0]));
		frontRightLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[1], globalFeetPos[1]));
		hindLeftLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[2], globalFeetPos[2]));
		hindRightLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[3], globalFeetPos[3]));
		stepLengthX = currentRobotSpeedX*stepTime*2.0;
		stepLengthY = currentRobotSpeedY*stepTime*2.0;
		switch(steppingLeg){
			case frontLeft:
				if(step.shiftCoM() && step.updateStep()){
					steppingLeg = leg.rearLeft;
					step = new Step(2);
				}
			break;
			case rearLeft:
				if(step.shiftCoM() && step.updateStep()){
					steppingLeg = leg.rearRight;
					step = new Step(3);
				}
			break;
			case frontRight:
				if(step.shiftCoM() && step.updateStep()){
					steppingLeg = leg.frontLeft;
					step = new Step(0);
				}
			break;
			case rearRight:
				if(step.shiftCoM() && step.updateStep()){
					steppingLeg = leg.frontRight;
					step = new Step(1);
				}
			break;
		}
		if(steppingLeg != lastSteppingLeg) lastGlobalStepCenter = globalStepCenter.clone();
		lastSteppingLeg = steppingLeg;
	}
	
	public void handleStanding(){
		updateGlobalRobotPos();
		globalCornerPos = Body.getGlobalCornerPos(localRobotPos, globalRobotPos);
		frontLeftLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[0], globalFeetPos[0]));
		frontRightLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[1], globalFeetPos[1]));
		hindLeftLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[2], globalFeetPos[2]));
		hindRightLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[3], globalFeetPos[3]));
		
	}
	
	public void handleTrottingLegs(){
		updateGlobalRobotPos();
		globalStepCenter = Body.getGlobalStepCenter(localRobotPos, globalRobotPos);
		globalCornerPos = Body.getGlobalCornerPos(localRobotPos, globalRobotPos);
		frontLeftLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[0], globalFeetPos[0]));
		frontRightLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[1], globalFeetPos[1]));
		hindLeftLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[2], globalFeetPos[2]));
		hindRightLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[3], globalFeetPos[3]));
		stepLengthX = currentRobotSpeedX*stepTime*2.0;
		stepLengthY = currentRobotSpeedY*stepTime*2.0;
		switch(steppingLeg){
			case rearRight:
			case frontLeft:
				if(step.balanceCoM() && step.updateFastStep() & stepMir.updateFastStep()){
					steppingLeg = leg.frontRight;
					step = new Step(2);
					stepMir = new Step(1);
				}
			break;
			case rearLeft:
			case frontRight:
				if(step.balanceCoM() && step.updateFastStep() & stepMir.updateFastStep()){
					steppingLeg = leg.frontLeft;
					step = new Step(0);
					stepMir = new Step(3);
				}
			break;
		}
		if(steppingLeg != lastSteppingLeg){
			lastGlobalStepCenter = globalStepCenter.clone();
		}
		lastSteppingLeg = steppingLeg;
	}
	
	public boolean home(){
		globalStepCenter = Body.getGlobalStepCenter(localRobotPos, globalRobotPos);
		globalCornerPos = Body.getGlobalCornerPos(localRobotPos, globalRobotPos);
		frontLeftLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[0], globalFeetPos[0]));
		frontRightLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[1], globalFeetPos[1]));
		hindLeftLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[2], globalFeetPos[2]));
		hindRightLeg.setFootPos(Body.getRelativeFootPos(globalCornerPos[3], globalFeetPos[3]));

		stepLengthX = 0;
		stepLengthY = 0;
		
		double footError = 0;
		boolean isHome = false;
		for(int i = 0; i < 3; i++){
			footError += Math.abs(globalStepCenter[i].x + globalStepCenter[i].y - globalFeetPos[i].x - globalFeetPos[i].y + globalFeetPos[i].z);
		}
		if(footError < .1) isHome = true;
		
		if(!isHome){
			switch(steppingLeg){
			case frontLeft:
				if(step.shiftCoM() && step.updateStep()){
					steppingLeg = leg.rearLeft;
					step = new Step(2);
				}
				break;
			case rearLeft:
				if(step.shiftCoM() && step.updateStep()){
					steppingLeg = leg.rearRight;
					step = new Step(3);
				}
				break;
			case frontRight:
				if(step.shiftCoM() && step.updateStep()){
					steppingLeg = leg.frontLeft;
					step = new Step(0);
				}
				break;
			case rearRight:
				if(step.shiftCoM() && step.updateStep()){
					steppingLeg = leg.frontRight;
					step = new Step(1);
				}
				break;
			}
			if(steppingLeg != lastSteppingLeg) lastGlobalStepCenter = globalStepCenter.clone();
			lastSteppingLeg = steppingLeg;
			return false;
		}
		else if(step.homeCoM()) return true;
		else return false;
		
	}
	
	public void render(Graphics g){
		int xpos[] = {(int) (globalCornerPos[0].x*Constants.PixelsPerIn + Constants.x0), (int) (globalCornerPos[1].x*Constants.PixelsPerIn + Constants.x0),
				(int) (globalCornerPos[3].x*Constants.PixelsPerIn + Constants.x0), (int) (globalCornerPos[2].x*Constants.PixelsPerIn + Constants.x0)};
		int ypos[] = {(int) (globalCornerPos[0].y*Constants.PixelsPerIn + Constants.y0), (int) (globalCornerPos[1].y*Constants.PixelsPerIn + Constants.y0),
				(int) (globalCornerPos[3].y*Constants.PixelsPerIn + Constants.y0), (int) (globalCornerPos[2].y*Constants.PixelsPerIn + Constants.y0)};
		
		int[] xposTri = {0,0,0,0};
		int[] yposTri = {0,0,0,0};
		
		for(int i = 0; i < 4; i++){
			if(i != steppingLeg.getLegNum()){
				xposTri[i] = (int) (globalFeetPos[i].x*Constants.PixelsPerIn + Constants.x0);
				yposTri[i] = (int) (globalFeetPos[i].y*Constants.PixelsPerIn + Constants.y0);
			}
			else if(steppingLeg.getLegNum() != 0){
				xposTri[i] = (int) (globalFeetPos[i-1].x*Constants.PixelsPerIn + Constants.x0);
				yposTri[i] = (int) (globalFeetPos[i-1].y*Constants.PixelsPerIn + Constants.y0);
			}
			else{
				xposTri[i] = (int) (globalFeetPos[i+1].x*Constants.PixelsPerIn + Constants.x0);
				yposTri[i] = (int) (globalFeetPos[i+1].y*Constants.PixelsPerIn + Constants.y0);
			}
		}

		//Draw the rotated rectangle
		g.setColor(Color.gray);
		g.fillPolygon(xpos, ypos, 4);
		g.setColor(Color.BLACK);
		g.drawPolygon(xpos, ypos, 4);
		g.fillOval((int) (globalRobotPos.x*Constants.PixelsPerIn + Constants.x0)-5, (int) (globalRobotPos.y*Constants.PixelsPerIn + Constants.y0)-5, 10, 10);//draw global pos
		//Draw Feet
		g.fillOval((int) (globalFeetPos[0].x*Constants.PixelsPerIn + Constants.x0)-5, (int) (globalFeetPos[0].y*Constants.PixelsPerIn + Constants.y0)-5, 10, 10);
		g.fillOval((int) (globalFeetPos[1].x*Constants.PixelsPerIn + Constants.x0)-5, (int) (globalFeetPos[1].y*Constants.PixelsPerIn + Constants.y0)-5, 10, 10);
		g.fillOval((int) (globalFeetPos[2].x*Constants.PixelsPerIn + Constants.x0)-5, (int) (globalFeetPos[2].y*Constants.PixelsPerIn + Constants.y0)-5, 10, 10);
		g.fillOval((int) (globalFeetPos[3].x*Constants.PixelsPerIn + Constants.x0)-5, (int) (globalFeetPos[3].y*Constants.PixelsPerIn + Constants.y0)-5, 10, 10);
		
		/*g.fillOval((int) (Body.getRelativeFootPos(globalCornerPos[0], globalFeetPos[0]).x*Constants.PixelsPerIn + Constants.x0)-5,
				(int) (Body.getRelativeFootPos(globalCornerPos[0], globalFeetPos[0]).y*Constants.PixelsPerIn + Constants.y0)-5, 10, 10);*/
		//draw stable base triangle
		g.drawPolygon(xposTri, yposTri, 4);
		if(steppingLeg == leg.frontLeft && HandleCoM.CoMStable(globalFeetPos[1], globalFeetPos[2], globalFeetPos[3], Body.getGlobalCoMPos(localRobotPos, globalRobotPos))) g.setColor(Color.WHITE);
		else if(steppingLeg == leg.frontRight && HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[2], globalFeetPos[3], Body.getGlobalCoMPos(localRobotPos, globalRobotPos))) g.setColor(Color.WHITE);
		else if(steppingLeg == leg.rearLeft && HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[1], globalFeetPos[3], Body.getGlobalCoMPos(localRobotPos, globalRobotPos))) g.setColor(Color.WHITE);
		else if(steppingLeg == leg.rearRight && HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[1], globalFeetPos[2], Body.getGlobalCoMPos(localRobotPos, globalRobotPos))) g.setColor(Color.WHITE);
		g.fillOval((int) (Body.getGlobalCoMPos(localRobotPos, globalRobotPos).x*Constants.PixelsPerIn + Constants.x0)-5,
				(int) (Body.getGlobalCoMPos(localRobotPos, globalRobotPos).y*Constants.PixelsPerIn + Constants.y0)-5, 10, 10);
		/*g.fillOval((int) (Body.getGlobalAdjustedCoMPos(localRobotPos, globalRobotPos, steppingLeg.getLegNum()).x*Constants.PixelsPerIn + Constants.x0)-5,
				(int) (Body.getGlobalAdjustedCoMPos(localRobotPos, globalRobotPos, steppingLeg.getLegNum()).y*Constants.PixelsPerIn + Constants.y0)-5, 10, 10);*/
	}
	
	/*Class for handling steps*/
	public class Step{
		
		private Timer stepTimer = new Timer();
		boolean startedStepping = false;
		boolean finishedStepping = false;
		int leg;
		private double shiftSpeed = 4.0;
		
		public Step(int leg){
			this.leg = leg;
		}
		
		public boolean updateStep(){
			if(!startedStepping) stepTimer.reset();//timer doesn't start until updateStep() is first called
			startedStepping = true;
			if(stepTimer.get() < .15){
				globalFeetPos[leg].z = stepHeight;
				//System.out.println("lifting");
			}
			else if(stepTimer.get() < .4){
				globalFeetPos[leg].x = lastGlobalStepCenter[leg].x + stepLengthX;
				globalFeetPos[leg].y = lastGlobalStepCenter[leg].y + stepLengthY;
				globalFeetPos[leg].z = stepHeight;
			}
			else if(stepTimer.get() < .5){
				globalFeetPos[leg].x = lastGlobalStepCenter[leg].x + stepLengthX;
				globalFeetPos[leg].y = lastGlobalStepCenter[leg].y + stepLengthY;
				globalFeetPos[leg].z = 0;
				//localRobotPos.y = -.5;
			}
			else{
				finishedStepping = true;
			}
			return finishedStepping;
		}
		
		public boolean updateFastStep(){
			if(!startedStepping) stepTimer.reset();//timer doesn't start until updateStep() is first called
			startedStepping = true;
			if(stepTimer.get() < .15){
				globalFeetPos[leg].x = lastGlobalStepCenter[leg].x + stepLengthX;
				globalFeetPos[leg].y = lastGlobalStepCenter[leg].y + stepLengthY;
				globalFeetPos[leg].z = stepHeight/2.0;
			}
			else if(stepTimer.get() < .4){
				globalFeetPos[leg].x = lastGlobalStepCenter[leg].x + stepLengthX;
				globalFeetPos[leg].y = lastGlobalStepCenter[leg].y + stepLengthY;
				globalFeetPos[leg].z = 0;
				//localRobotPos.y = -.5;
			}
			else{
				finishedStepping = true;
			}
			return finishedStepping;
		}
		
		public boolean shiftCoM(){
			double speed = shiftSpeed/updateRate;
			Position adjustedCoMPos = Body.getGlobalAdjustedCoMPos(localRobotPos, globalRobotPos, steppingLeg.getLegNum());
			switch(steppingLeg){
			case frontLeft:
				if(!HandleCoM.CoMStable(globalFeetPos[1], globalFeetPos[2], globalFeetPos[3], adjustedCoMPos)){
					double dist = Trig.dist(adjustedCoMPos, globalFeetPos[3]);
					localRobotPos.y += speed*(globalFeetPos[3].y - adjustedCoMPos.y)/dist;
					localRobotPos.x += speed*(globalFeetPos[3].x - adjustedCoMPos.x)/dist;
				}
				else return true;
				break;
			case frontRight:
				if(!HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[2], globalFeetPos[3], adjustedCoMPos)){
					double dist = Trig.dist(adjustedCoMPos, globalFeetPos[2]);
					localRobotPos.y += speed*(globalFeetPos[2].y - adjustedCoMPos.y)/dist;
					localRobotPos.x += speed*(globalFeetPos[2].x - adjustedCoMPos.x)/dist;
				}
				else return true;
				break;
			case rearLeft:
				if(!HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[1], globalFeetPos[3], adjustedCoMPos)){
					double dist = Trig.dist(adjustedCoMPos, globalFeetPos[1]);
					localRobotPos.y += speed*(globalFeetPos[1].y - adjustedCoMPos.y)/dist;
					localRobotPos.x += speed*(globalFeetPos[1].x - adjustedCoMPos.x)/dist;
				}
				else return true;
				break;
			case rearRight:
				if(!HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[1], globalFeetPos[2], adjustedCoMPos)){
					double dist = Trig.dist(adjustedCoMPos, globalFeetPos[0]);
					localRobotPos.y += speed*(globalFeetPos[0].y - adjustedCoMPos.y)/dist;
					localRobotPos.x += speed*(globalFeetPos[0].x - adjustedCoMPos.x)/dist;
				}
				else return true;
				break;
			}
			return false;
		}
		
		public boolean balanceCoM(){
			double speed = shiftSpeed/updateRate;
			Position CoMPos = Body.getGlobalCoMPos(localRobotPos, globalRobotPos);
			Position frontBalanceLimit = Body.getGlobalAdjustedCoMPos(localRobotPos, globalRobotPos, steppingLeg.getLegNum());
			Position rearBalanceLimit = Body.getGlobalAdjustedCoMPos(localRobotPos, globalRobotPos, 3 - steppingLeg.getLegNum());
			switch(steppingLeg){
			case rearRight:
			case frontLeft:
				if(!HandleCoM.CoMStable(globalFeetPos[1], globalFeetPos[2], globalFeetPos[3], CoMPos)){
					double dist = Trig.dist(CoMPos, globalFeetPos[3]);
					localRobotPos.y += speed*(globalFeetPos[3].y - CoMPos.y)/dist;
					localRobotPos.x += speed*(globalFeetPos[3].x - CoMPos.x)/dist;
				}
				else{
					double dist = Trig.dist(CoMPos, globalFeetPos[3]);
					localRobotPos.y -= speed*(globalFeetPos[3].y - CoMPos.y)/dist;
					localRobotPos.x -= speed*(globalFeetPos[3].x - CoMPos.x)/dist;
				}
				if(HandleCoM.CoMStable(globalFeetPos[1], globalFeetPos[2], globalFeetPos[3], frontBalanceLimit) &&
						!HandleCoM.CoMStable(globalFeetPos[1], globalFeetPos[2], globalFeetPos[3], rearBalanceLimit));
				else return true;
				break;
			case rearLeft:
			case frontRight:
				if(!HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[2], globalFeetPos[3], CoMPos)){
					double dist = Trig.dist(CoMPos, globalFeetPos[2]);
					localRobotPos.y += speed*(globalFeetPos[2].y - CoMPos.y)/dist;
					localRobotPos.x += speed*(globalFeetPos[2].x - CoMPos.x)/dist;
				}
				else{
					double dist = Trig.dist(CoMPos, globalFeetPos[2]);
					localRobotPos.y -= speed*(globalFeetPos[2].y - CoMPos.y)/dist;
					localRobotPos.x -= speed*(globalFeetPos[2].x - CoMPos.x)/dist;
				}
				if(HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[2], globalFeetPos[3], frontBalanceLimit) &&
						!HandleCoM.CoMStable(globalFeetPos[0], globalFeetPos[2], globalFeetPos[3], rearBalanceLimit));
				else return true;
				break;
				
			}
			return false;
		}
		
		public boolean homeCoM(){
			if(Math.abs(localRobotPos.y) > .1) localRobotPos.y -= Math.signum(localRobotPos.y)*shiftSpeed/updateRate;
			else localRobotPos.y = 0;
			if(Math.abs(localRobotPos.y) > .1) localRobotPos.x -= Math.signum(localRobotPos.x)*shiftSpeed/updateRate;
			else localRobotPos.x = 0;
			if(localRobotPos.y == 0 && localRobotPos.x == 0) return true;
			else return false;
		}
		
	}
	

}

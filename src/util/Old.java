package util;

import input.KeyManager;
import motion.RobotMotion.State;

public class Old {

}

/*KeyManager.tick();
if(KeyManager.numup) {
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

KeyManager.tick();
		if(KeyManager.num5) currentState = State.stopped;
		else if(KeyManager.i) currentState = State.trotting;
		else if(KeyManager.numup || KeyManager.numdown || KeyManager.numleft || KeyManager.numright) currentState = State.walking;
		//else if(KeyManager.j || KeyManager.k || KeyManager.l || KeyManager.i || KeyManager.o || KeyManager.u) currentState = State.standing;
		else if(KeyManager.space) end = true;
		else if(KeyManager.s) currentState = State.standing;*/
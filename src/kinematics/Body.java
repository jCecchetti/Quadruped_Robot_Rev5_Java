package kinematics;

import util.Constants;
import util.Matrix;
import util.Position;

public class Body {
	
	private static Position[] localCornerPos = {new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0)};
	private static Position[] globalCornerPos = {new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0)};
	private static Position[] globalStepCenter = {new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0),new Position(0,0,0,0,0,0)};
	private static double[][] localCoMBox = {{Constants.COMBOX, Constants.COMBOX, 0},
											{Constants.COMBOX, -Constants.COMBOX, 0},
											{-Constants.COMBOX, Constants.COMBOX, 0},
											{-Constants.COMBOX, -Constants.COMBOX, 0}};
	
	public static Position[] getLocalCornerPos(Position localBodyPos){
		double[][] unrotatedLocalCornerPos = {{Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0},
										{Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0},
										{-Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0},
										{-Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0}};
		
		double[][] pitchRotationMatrix = {{Math.cos(Math.toRadians(localBodyPos.pitch)), 0, -Math.sin(Math.toRadians(localBodyPos.pitch))},
										 {0, 1, 0},
										 {Math.sin(Math.toRadians(localBodyPos.pitch)), 0, Math.cos(Math.toRadians(localBodyPos.pitch))}};
		double[][] rollRotationMatrix = {{1, 0, 0},
										  {0, Math.cos(Math.toRadians(localBodyPos.roll)), Math.sin(Math.toRadians(localBodyPos.roll))},
										  {0, -Math.sin(Math.toRadians(localBodyPos.roll)), Math.cos(Math.toRadians(localBodyPos.roll))}};
		double[][] yawRotationMatrix = {{Math.cos(Math.toRadians(localBodyPos.yaw)), -Math.sin(Math.toRadians(localBodyPos.yaw)), 0},
									    {Math.sin(Math.toRadians(localBodyPos.yaw)), Math.cos(Math.toRadians(localBodyPos.yaw)), 0},
									    {0, 0, 1}};
		for(int i = 0; i < 4; i++){
			unrotatedLocalCornerPos[i] = Matrix.multiply(rollRotationMatrix, unrotatedLocalCornerPos[i]);
			unrotatedLocalCornerPos[i] = Matrix.multiply(pitchRotationMatrix, unrotatedLocalCornerPos[i]);
			double[] rotatedLocalCornerPos = Matrix.multiply(yawRotationMatrix, unrotatedLocalCornerPos[i]);
			localCornerPos[i] = new Position(rotatedLocalCornerPos[0] + localBodyPos.x, rotatedLocalCornerPos[1] + localBodyPos.y,
					rotatedLocalCornerPos[2] + localBodyPos.z, localBodyPos.roll, localBodyPos.pitch, localBodyPos.yaw);
			//System.out.println(rotatedLocalCornerPos[0]);
		}
		return localCornerPos;
	}
	
	public static Position[] getGlobalCornerPos(Position localBodyPos, Position globalBodyPos){
		Position[] untranslatedCornerPos = getLocalCornerPos(localBodyPos);
		for(int i = 0; i < 4; i++){
			globalCornerPos[i] = new Position(untranslatedCornerPos[i].x + globalBodyPos.x, untranslatedCornerPos[i].y + globalBodyPos.y,
					untranslatedCornerPos[i].z + globalBodyPos.z, localBodyPos.roll, localBodyPos.pitch, localBodyPos.yaw);
		}
		return globalCornerPos;
	}
	
	public static Position[] getGlobalStepCenter(Position localBodyPos, Position globalBodyPos){
		
		double[][] unrotatedStepCenter = {{Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0},
				{Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0},
				{-Constants.BODYLENGTH/2, Constants.BODYWIDTH/2, 0},
				{-Constants.BODYLENGTH/2, -Constants.BODYWIDTH/2, 0}};
		
		double[][] yawRotationMatrix = {{Math.cos(Math.toRadians(localBodyPos.yaw)), -Math.sin(Math.toRadians(localBodyPos.yaw)), 0},
				{Math.sin(Math.toRadians(localBodyPos.yaw)), Math.cos(Math.toRadians(localBodyPos.yaw)), 0},
				{0, 0, 1}};
		
		for(int i = 0; i < 4; i++){
			double[] rotatedStepCenter = Matrix.multiply(yawRotationMatrix, unrotatedStepCenter[i]);
			globalStepCenter[i] = new Position(rotatedStepCenter[0] + globalBodyPos.x, rotatedStepCenter[1] + globalBodyPos.y,
					rotatedStepCenter[2] + globalBodyPos.z, localBodyPos.roll, localBodyPos.pitch, localBodyPos.yaw);
		}
		
		return globalStepCenter;
	}
	
	public static Position getGlobalCoMPos(Position localBodyPos, Position globalBodyPos){
		Position globalCoMPos = new Position(globalBodyPos.x + localBodyPos.x, globalBodyPos.y + localBodyPos.y, globalBodyPos.z + localBodyPos.z, 0, 0, 0);
		return globalCoMPos;
	}
	
	public static Position getGlobalAdjustedCoMPos(Position localBodyPos, Position globalBodyPos, int corner){
		double[][] yawRotationMatrix = {{Math.cos(Math.toRadians(localBodyPos.yaw)), -Math.sin(Math.toRadians(localBodyPos.yaw)), 0},
			    {Math.sin(Math.toRadians(localBodyPos.yaw)), Math.cos(Math.toRadians(localBodyPos.yaw)), 0},
			    {0, 0, 1}};
		Position globalCoMPos = getGlobalCoMPos(localBodyPos, globalBodyPos);
		
		double[] adjustedCoMPos = Matrix.multiply(yawRotationMatrix, localCoMBox[corner]);//get position of adjusted CoM opposite of the stepping leg
		return new Position(adjustedCoMPos[0] + globalCoMPos.x, adjustedCoMPos[1] + globalCoMPos.y, 0, 0, 0, 0);
	}
	
	public static Position getRelativeFootPos(Position globalCornerPos, Position globalFootPos){
		
		double[] unrotatedLocalFootPos = {globalFootPos.x - globalCornerPos.x, globalFootPos.y - globalCornerPos.y, globalFootPos.z - globalCornerPos.z};
		
		double[][] rollRotationMatrix = {{1, 0, 0},
				  {0, Math.cos(Math.toRadians(-globalCornerPos.roll)), Math.sin(Math.toRadians(-globalCornerPos.roll))},
				  {0, -Math.sin(Math.toRadians(-globalCornerPos.roll)), Math.cos(Math.toRadians(-globalCornerPos.roll))}};
		double[][] pitchRotationMatrix = {{Math.cos(Math.toRadians(-globalCornerPos.pitch)), 0, -Math.sin(Math.toRadians(-globalCornerPos.pitch))},
				 {0, 1, 0},
				 {Math.sin(Math.toRadians(-globalCornerPos.pitch)), 0, Math.cos(Math.toRadians(-globalCornerPos.pitch))}};
		double[][] yawRotationMatrix = {{Math.cos(Math.toRadians(-globalCornerPos.yaw)), -Math.sin(Math.toRadians(-globalCornerPos.yaw)), 0},
			    {Math.sin(Math.toRadians(-globalCornerPos.yaw)), Math.cos(Math.toRadians(-globalCornerPos.yaw)), 0},
			    {0, 0, 1}};
		
		unrotatedLocalFootPos = Matrix.multiply(rollRotationMatrix, unrotatedLocalFootPos);
		unrotatedLocalFootPos = Matrix.multiply(pitchRotationMatrix, unrotatedLocalFootPos);
		unrotatedLocalFootPos = Matrix.multiply(yawRotationMatrix, unrotatedLocalFootPos);
		

		Position relativeFootPos = new Position(unrotatedLocalFootPos[0], unrotatedLocalFootPos[1], unrotatedLocalFootPos[2],0,0,0);
		
		return relativeFootPos;
	}
	
	public static Position getGlobalFootPos(Position globalCornerPos, Position relativeFootPos){
		return relativeFootPos;
	}
	
}

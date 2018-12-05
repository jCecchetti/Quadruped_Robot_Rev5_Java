package motion;

import java.awt.Graphics;

import util.Matrix;
import util.Position;

public class HandleCoM {
	
	public static boolean CoMStable(Position a, Position b, Position c, Position s){
		double as_x = s.x-a.x;
		double as_y = s.y-a.y;
		
		boolean s_ab = (b.x-a.x)*as_y - (b.y-a.y)*as_x > 0;
		
		if((c.x-a.x)*as_y - (c.y-a.y)*as_x > 0 == s_ab) return false;
		if((c.x-b.x)*(s.y-b.y) - (c.y-b.y)*(s.x-b.x) > 0 != s_ab) return false;
		return true;
	}
	/*public static boolean CoMStable(Position diagonalLeg1, Position diagonalLeg2, Position farLeg, Position CoMPos){
		double a = (Matrix.determinate(CoMPos.x - farLeg.x, CoMPos.y - farLeg.y, diagonalLeg2.x - farLeg.x, diagonalLeg2.y - farLeg.y) - 
				Matrix.determinate(farLeg.x, farLeg.y, diagonalLeg2.x - farLeg.x, diagonalLeg2.y - farLeg.y))/
				Matrix.determinate(diagonalLeg1.x - farLeg.x, diagonalLeg1.y - farLeg.y, diagonalLeg2.x - farLeg.x, diagonalLeg2.y - farLeg.y);
		double b = (Matrix.determinate(CoMPos.x - farLeg.x, CoMPos.y - farLeg.y, diagonalLeg1.x - farLeg.x, diagonalLeg1.y - farLeg.y) - 
				Matrix.determinate(farLeg.x, farLeg.y, diagonalLeg1.x - farLeg.x, diagonalLeg1.y - farLeg.y))/
				Matrix.determinate(diagonalLeg1.x - farLeg.x, diagonalLeg1.y - farLeg.y, diagonalLeg2.x - farLeg.x, diagonalLeg2.y - farLeg.y);
		
		return (a > 0 && b > 0 && a + b < 1);
	}*/
	

	public static void render(Graphics g){
		
	}

}

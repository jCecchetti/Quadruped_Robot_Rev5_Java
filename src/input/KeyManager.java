package input;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

public class KeyManager implements KeyListener {
	
	public static boolean[] keys;
	public static boolean up, down, left, right, numup, numdown, numleft, numright, num5,
	q, a, z, w, s, x, e, d, c, r, f, v, t, g, b, y, h, n, u, j, m, i, k, o, l, comma, period, shift, space;
	
	
	public KeyManager(){
		keys = new boolean[256];
	}
	
	public static void tick(){
		
		up = keys[KeyEvent.VK_UP];
		down = keys[KeyEvent.VK_DOWN];
		left = keys[KeyEvent.VK_LEFT];
		right = keys[KeyEvent.VK_RIGHT];	
		
		numup = keys[KeyEvent.VK_NUMPAD8];
		numdown = keys[KeyEvent.VK_NUMPAD2];
		numleft = keys[KeyEvent.VK_NUMPAD4];
		numright = keys[KeyEvent.VK_NUMPAD6];
		num5 = keys[KeyEvent.VK_NUMPAD5];
		
		q = keys[KeyEvent.VK_Q];
		a = keys[KeyEvent.VK_A];
		z = keys[KeyEvent.VK_Z];
		w = keys[KeyEvent.VK_W];
		s = keys[KeyEvent.VK_S];
		x = keys[KeyEvent.VK_X];
		e = keys[KeyEvent.VK_E];
		d = keys[KeyEvent.VK_D];
		c = keys[KeyEvent.VK_C];
		
		r = keys[KeyEvent.VK_R];
		f = keys[KeyEvent.VK_F];
		v = keys[KeyEvent.VK_V];
		t = keys[KeyEvent.VK_T];
		g = keys[KeyEvent.VK_G];
		b = keys[KeyEvent.VK_B];
		y = keys[KeyEvent.VK_Y];
		h = keys[KeyEvent.VK_H];
		n = keys[KeyEvent.VK_N];
		
		i = keys[KeyEvent.VK_I];
		k = keys[KeyEvent.VK_K];
		j = keys[KeyEvent.VK_J];
		l = keys[KeyEvent.VK_L];
		
		o = keys[KeyEvent.VK_O];
		
		shift = keys[KeyEvent.VK_SHIFT];

		space = keys[KeyEvent.VK_SPACE];
	}
	
	@Override
	public void keyPressed(KeyEvent e) {
		keys[e.getKeyCode()] = true;
	}

	@Override
	public void keyReleased(KeyEvent e) {
		keys[e.getKeyCode()] = false;
	}

	@Override
	public void keyTyped(KeyEvent e) {
		
		
	}
	
}

package frc.robot.util;

import java.util.stream.Stream;

public class Color {
	
	public static final Color WHITE = new Color(255, 255, 255, 255);
	
	public static final Color BLACK = new Color(0, 0, 0, 255);
	
	public static final Color FRC_RED = new Color(237, 28, 36, 255);
	
	public static final Color FRC_BLUE = new Color(0, 101, 179, 255);
	
	protected final int red;
	
	protected final int green;
	
	protected final int blue;
	
	protected final int alpha;
	
	public Color(int red, int green, int blue, int alpha) {
		
		this.red = red;
		this.green = green;
		this.blue = blue;
		this.alpha = alpha;
		
	}
	
	public String toARGBHexString() {
		
		return Stream.of(this.alpha, this.red, this.green, this.blue)
			.map(Integer::toHexString)
			.reduce(String::join)
			.get();
		
	}
	
	public int toARGBInt() {
		
		return Integer.parseInt(this.toARGBHexString(), 16);
		
	}
	
}

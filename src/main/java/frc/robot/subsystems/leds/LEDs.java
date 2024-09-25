package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.leds.LEDConstants;
import frc.robot.utils.leds.LEDConstants.ImageStates;
import frc.robot.utils.leds.LEDConstants.LEDStates;
import frc.robot.utils.leds.LEDConstants.PanelOrientation;
import frc.robot.utils.maths.CommonMath;
import frc.robot.utils.maths.TimeUtil;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.Collections;
import java.util.HashMap;
import java.nio.file.Files;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;

public class LEDs extends SubsystemChecker {
	// Basic 5x7 bitmap for ASCII characters A-Z
	private static final Map<Character, boolean[][]> CHAR_MAP = new HashMap<>();
	static {
		// Character 'A'
		CHAR_MAP.put('A', new boolean[][] {
				{ false, false, true, true, true, true, false, false
				}, { false, true, false, false, false, false, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, true, true, true, true, true, true, true, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}
		});
		// Character 'B'
		CHAR_MAP.put('B',
				new boolean[][] { { true, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, true, true, true, true, true, true, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, true, true, true, true, true, true, false
				}
				});
		// Character 'C'
		CHAR_MAP.put('C',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character 'D'
		CHAR_MAP.put('D',
				new boolean[][] { { true, true, true, true, true, true, false, false
				}, { true, false, false, false, false, false, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, true, false
				}, { true, true, true, true, true, true, true, false
				}
				});
		// Character 'E'
		CHAR_MAP.put('E',
				new boolean[][] { { true, true, true, true, true, true, true, true
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, true, true, true, true, true, true, true
				}
				});
		// Character 'F'
		CHAR_MAP.put('F',
				new boolean[][] { { true, true, true, true, true, true, true, true
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}
				});
		// Character 'G'
		CHAR_MAP.put('G',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, true, true, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character 'H'
		CHAR_MAP.put('H', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, true, true, true, true, true, true, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}
		});
		// Character 'I'
		CHAR_MAP.put('I',
				new boolean[][] { { true, true, true, true, true, true, true, true
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { true, true, true, true, true, true, true, true
				}
				});
		// Character 'J'
		CHAR_MAP.put('J', new boolean[][] {
				{ false, false, false, false, true, true, true, true
				}, { false, false, false, false, false, false, true, true
				}, { false, false, false, false, false, false, true, true
				}, { false, false, false, false, false, false, true, true
				}, { false, false, false, false, false, false, true, true
				}, { true, false, false, false, false, false, true, true
				}, { true, false, false, false, false, false, true, true
				}, { false, true, true, true, true, true, false, false
				}
		});
		// Character 'K'
		CHAR_MAP.put('K', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, true, false
				}, { true, false, false, false, false, true, false, false
				}, { true, true, true, true, true, false, false, false
				}, { true, false, false, false, false, true, false, false
				}, { true, false, false, false, false, false, true, false
				}, { true, false, false, false, false, false, false, true
				}
		});
		// Character 'L'
		CHAR_MAP.put('L', new boolean[][] {
				{ true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, true, true, true, true, true, true, true
				}
		});
		// Character 'M'
		CHAR_MAP.put('M', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, true, false, false, false, false, true, true
				}, { true, false, true, false, false, true, false, true
				}, { true, false, false, true, true, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}
		});
		// Character 'N'
		CHAR_MAP.put('N', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, true, false, false, false, false, false, true
				}, { true, false, true, false, false, false, false, true
				}, { true, false, false, true, false, false, false, true
				}, { true, false, false, false, true, false, false, true
				}, { true, false, false, false, false, true, false, true
				}, { true, false, false, false, false, false, true, true
				}
		});
		// Character 'O'
		CHAR_MAP.put('O',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character 'P'
		CHAR_MAP.put('P',
				new boolean[][] { { true, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}
				});
		// Character 'Q'
		CHAR_MAP.put('Q',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, true, false, true
				}, { true, false, false, false, false, false, true, false
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character 'R'
		CHAR_MAP.put('R',
				new boolean[][] { { true, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}
				});
		// Character 'S'
		CHAR_MAP.put('S', new boolean[][] {
				{ false, false, true, true, true, true, true, false
				}, { false, true, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, false
				}, { false, true, true, true, true, true, true, true
				}, { false, false, false, false, false, false, false, true
				}, { false, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
		});
		// Character 'T'
		CHAR_MAP.put('T',
				new boolean[][] { { true, true, true, true, true, true, true, true
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}
				});
		// Character 'U'
		CHAR_MAP.put('U', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
		});
		// Character 'V'
		CHAR_MAP.put('V', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, false, false, false, false, true, false
				}, { false, true, false, false, false, false, true, false
				}, { false, false, true, false, false, true, false, false
				}, { false, false, true, false, false, true, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}
		});
		// Character 'W'
		CHAR_MAP.put('W', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, true, true, false, false, true
				}, { true, false, true, false, false, true, false, true
				}, { true, true, false, false, false, false, true, true
				}, { true, false, false, false, false, false, false, true
				}
		});
		// Character 'X'
		CHAR_MAP.put('X', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, false, false, false, false, true, false
				}, { false, false, true, false, false, true, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, true, false, false, true, false, false
				}, { false, true, false, false, false, false, true, false
				}, { true, false, false, false, false, false, false, true
				}
		});
		// Character 'Y'
		CHAR_MAP.put('Y', new boolean[][] {
				{ true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, false, false, false, false, true, false
				}, { false, false, true, false, false, true, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}
		});
		// Character 'Z'
		CHAR_MAP.put('Z',
				new boolean[][] { { true, true, true, true, true, true, true, true
				}, { false, false, false, false, false, true, true, false
				}, { false, false, false, false, true, false, false, false
				}, { false, false, false, true, false, false, false, false
				}, { false, false, true, false, false, false, false, false
				}, { false, true, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, true, true, true, true, true, true, true
				}
				});
		// Character '0'
		CHAR_MAP.put('0',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, true, true
				}, { true, false, false, false, false, true, false, true
				}, { true, false, false, false, true, false, false, true
				}, { true, false, false, true, false, false, false, true
				}, { true, false, true, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character '1'
		CHAR_MAP.put('1', new boolean[][] {
				{ false, false, true, false, false, false, false, false
				}, { false, true, true, false, false, false, false, false
				}, { true, false, true, false, false, false, false, false
				}, { false, false, true, false, false, false, false, false
				}, { false, false, true, false, false, false, false, false
				}, { false, false, true, false, false, false, false, false
				}, { false, false, true, false, false, false, false, false
				}, { true, true, true, true, true, true, true, true
				}
		});
		// Character '2'
		CHAR_MAP.put('2',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { false, false, false, false, false, false, true, false
				}, { false, false, false, false, false, true, false, false
				}, { false, false, false, false, true, false, false, false
				}, { false, false, false, true, false, false, false, false
				}, { false, false, true, false, false, false, false, false
				}, { true, true, true, true, true, true, true, true
				}
				});
		// Character '3'
		CHAR_MAP.put('3',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { false, false, false, false, false, false, true, false
				}, { false, false, false, false, false, true, false, false
				}, { false, false, false, false, false, false, true, false
				}, { false, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character '4'
		CHAR_MAP.put('4', new boolean[][] {
				{ false, false, false, false, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, true, false, true, false, false, false
				}, { false, true, false, false, true, false, false, false
				}, { true, true, true, true, true, true, true, true
				}, { false, false, false, false, true, false, false, false
				}, { false, false, false, false, true, false, false, false
				}, { false, false, false, false, true, false, false, false
				}
		});
		// Character '5'
		CHAR_MAP.put('5',
				new boolean[][] { { true, true, true, true, true, true, true, true
				}, { true, false, false, false, false, false, false, false
				}, { true, false, false, false, false, false, false, false
				}, { true, true, true, true, true, true, true, false
				}, { false, false, false, false, false, false, false, true
				}, { false, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character '6'
		CHAR_MAP.put('6',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, false
				}, { true, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character '7'
		CHAR_MAP.put('7',
				new boolean[][] { { true, true, true, true, true, true, true, true
				}, { false, false, false, false, false, false, false, true
				}, { false, false, false, false, false, false, true, false
				}, { false, false, false, false, false, true, false, false
				}, { false, false, false, false, true, false, false, false
				}, { false, false, false, true, false, false, false, false
				}, { false, false, false, true, false, false, false, false
				}, { false, false, false, true, false, false, false, false
				}
				});
		// Character '8'
		CHAR_MAP.put('8',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character '9'
		CHAR_MAP.put('9',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { true, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, true
				}, { false, false, false, false, false, false, false, true
				}, { false, false, false, false, false, false, false, true
				}, { false, true, true, true, true, true, true, false
				}
				});
		// Character ' '
		CHAR_MAP.put(' ', new boolean[][] {
				{ false, false, false, false, false, false, false, false
				}, { false, false, false, false, false, false, false, false
				}, { false, false, false, false, false, false, false, false
				}, { false, false, false, false, false, false, false, false
				}, { false, false, false, false, false, false, false, false
				}, { false, false, false, false, false, false, false, false
				}, { false, false, false, false, false, false, false, false
				}, { false, false, false, false, false, false, false, false
				}
		});
		// Character '!'
		CHAR_MAP.put('!',
				new boolean[][] { { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, false, false, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}
				});
		// Character '?'
		CHAR_MAP.put('?',
				new boolean[][] { { false, true, true, true, true, true, true, false
				}, { true, false, false, false, false, false, false, true
				}, { false, false, false, false, false, true, true, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, false, false, false, false, false
				}, { false, false, false, true, true, false, false, false
				}, { false, false, false, true, true, false, false, false
				}
				});
	}
	public static AddressableLED leds;
	public static AddressableLEDBuffer ledBuffer;
	public static AddressableLEDSim ledSim;
	public static String gifStorage;
	public static boolean override = false, debounce = false;
	public static LEDStates[] currentLEDState = { LEDStates.SOLID_COLOR, LEDStates.SOLID_COLOR
	};
	private int[][] color = new int[][] { LEDConstants.whiteRGB,
			LEDConstants.whiteRGB
	};
	public int[] flashRateMs = new int[] { 1000, 1000
	};
	private double[] brightness = new double[] { 1, 1
	}; //1 is max
	private double currentTimeMs = TimeUtil.getLogTimeSeconds() * 1000.0;
	private int[][] altColor = new int[][] { LEDConstants.redRGB,
			LEDConstants.greenRGB
	};
	//Rainbow variables
	private double[] m_firstHue = new double[] { 0, 0
	}; //Holds the current hue of the rainbow
	//Wave variables
	private double[] wavePhaseOffset = new double[] { 0, 0
	}; //Holds the current phase of the wave
	//Breathing variables
	private double[] breathingPhaseOffset = new double[] { 0, 0
	}; //Holds the current phase of the breathing
	//Gif variables
	public static ImageStates[] currentImageState = new ImageStates[] {
			ImageStates.debug, ImageStates.debug
	}; //Holds the current image state
	private static int[] currentImageIndex = new int[] { 0, 0
	}; //Holds the current image index WITHIN the image state
	private static double[] lastUpdateTimeMs = new double[] {
			Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY
	}; //Holds the last time the gif (or fire) was updated
	//Fire variables ONLY USABLE WITH ONE DISPLAY
	private static final int[] heat = new int[(int) LEDConstants.ledBufferLength];
	private static int COOLING = 55;
	private static int SPARKING = 120;
	private static boolean reverseDirection = true;
	//Text variables
	private static String[] fontString = new String[] { "Monospaced",
			"Monospaced"
	};
	private static String[] text = new String[] { "ROBOT?", "135"
	};
	private double[] offsetX = new double[] { 0, 0
	};
	public static int textIndex = 0;

	public LEDs() {
		switch (Constants.currentMode) {
		case REAL:
			gifStorage = "/U/images";
			break;
		default:
			ledSim = new AddressableLEDSim(leds);
			gifStorage = "src\\main\\java\\frc\\robot\\utils\\leds\\images";
			break;
		}
		//creates LED objects (the actual LEDs, and a buffer that stores data to be sent to them)
		leds = new AddressableLED((int) LEDConstants.ledPort);
		ledBuffer = new AddressableLEDBuffer((int) LEDConstants.ledBufferLength);
		//sets length of the LED strip to buffer length
		leds.setLength(ledBuffer.getLength());
		//starts LED strips
		leds.start(); //FOR THE LOVE OF GOD PLEASE REMEMBER THIS IF YOU'RE GONNA CODE YOUR OWN SUBSYSTEM I SPENT LIKE 6 HOURS TROUBLESHOOTING AND IT DIDNT WORK BECAUSE OF THIS -N
		//if the robot is a simulation, create a simulation for the addressableLEDs
		LEDConstants.imageLedStates = preprocessImages(LEDConstants.imageList);
	}

	int timeCheck = 0;

	@Override
	public void periodic() {
		// called every 20ms
		currentTimeMs = TimeUtil.getLogTimeSeconds() * 1000.0;
		timeCheck += 20;
		runHandler();
		for (int i = 0; i < currentLEDState.length; i++) {
			switch (currentLEDState[i]) {
			case OFF:
				color[i] = LEDConstants.disabledRGB;
				setSolidColor(i);
				break;
			case SOLID_COLOR:
				setSolidColor(i);
				break;
			case RAINBOW:
				setRainbow(i);
				break;
			case SINE_WAVE:
				setWave(i);
				break;
			case WAVE2:
				setWave2(i);
				break;
			case BREATHING:
				setBreathing(i);
				break;
			case GIF:
				setGif(i);
				break;
			case FIRE:
				setFire(i);
				break;
			case TEXT:
				setText(i);
				break;
			}
		}
		leds.setData(ledBuffer);
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
	}

	private double getStartVal(int panelIndex) {
		switch (panelIndex) {
		case 0:
			return 0;
		case 1:
			return LEDConstants.ledBufferCutoff;
		default:
			return 0;
		}
	}

	private double getEndVal(int panelIndex) {
		switch (panelIndex) {
		case 0:
			return LEDConstants.ledBufferCutoff;
		case 1:
			return LEDConstants.ledBufferLength;
		default:
			return 1;
		}
	}

	private double getLength(int panelIndex) {
		switch (panelIndex) {
		case 0:
			return LEDConstants.ledBufferCutoff;
		case 1:
			return LEDConstants.ledBufferLength - LEDConstants.ledBufferCutoff;
		default:
			return 1;
		}
	}

	/**
	 * Sets the LEDs to a solid color INTERNAL ONLY
	 */
	private void setSolidColor(int panelIndex) {
		double startVal = getStartVal(panelIndex);
		double endVal = getEndVal(panelIndex);
		for (var i = (int) startVal; i < endVal; i++) {
			ledBuffer.setRGB(i,
					(int) (color[panelIndex][0] * brightness[panelIndex]),
					(int) (color[panelIndex][1] * brightness[panelIndex]),
					(int) (color[panelIndex][2] * brightness[panelIndex]));
		}
	}

	/**
	 * Sets the LEDs to a rainbow INTERNAL ONLY
	 */
	private void setRainbow(int panelIndex) {
		int currentHue;
		double startVal = getStartVal(panelIndex);
		double endVal = getEndVal(panelIndex);
		double length = getLength(panelIndex);
		for (int index = (int) startVal; index < endVal; index++) {
			int pixelIndex = getLedIndex(index);
			currentHue = (int) (m_firstHue[panelIndex] + (index * 180 / length))
					% 180;
			ledBuffer.setHSV(pixelIndex, currentHue, 255, 128);
		}
		m_firstHue[panelIndex] = Math.round((m_firstHue[panelIndex]
				+ (3.6 / (flashRateMs[panelIndex] / 1000.0))));
		m_firstHue[panelIndex] %= 180;
	}

	/**
	 * Sets the LEDs to a sine wave of 1 color INTERNAL ONLY
	 */
	private void setWave(int panelIndex) {
		double startVal = getStartVal(panelIndex);
		double endVal = getEndVal(panelIndex);
		double length = getLength(panelIndex);
		double phaseInc = (10.25 * (length / 512))
				/ (flashRateMs[panelIndex] / 1000.0); // 10.25 was found to be this functions period FOR 512 PIXELS!
		for (int i = (int) startVal; i < endVal; i++) {
			double x = (wavePhaseOffset[panelIndex] + i) * (2.0 * Math.PI)
					/ length;
			double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0)
					/ 2.0;
			if (Double.isNaN(ratio)) {
				ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent)
						+ 1.0) / 2.0;
			}
			if (Double.isNaN(ratio)) {
				ratio = 0.5;
			}
			int pixelIndex = getLedIndex(i);
			int red = (int) (color[panelIndex][0] * ratio);
			int green = (int) (color[panelIndex][1] * ratio);
			int blue = (int) (color[panelIndex][2] * ratio);
			ledBuffer.setRGB(pixelIndex, (int) (red * brightness[panelIndex]),
					(int) (green * brightness[panelIndex]),
					(int) (blue * brightness[panelIndex]));
		}
		wavePhaseOffset[panelIndex] += phaseInc;
		wavePhaseOffset[panelIndex] %= length;
	}

	/**
	 * Sets the LEDs to a sine wave of 2 colors INTERNAL ONLY
	 */
	private void setWave2(int panelIndex) {
		double startVal = getStartVal(panelIndex);
		double endVal = getEndVal(panelIndex);
		double length = getLength(panelIndex);
		double constantMultiplier = 10.25 * (length / 512);
		double phaseInc = constantMultiplier / (flashRateMs[panelIndex] / 1000.0); // 10.25 was found to be this functions period FOR 512 PIXELS!
		for (int i = (int) startVal; i < endVal; i++) {
			double x = (wavePhaseOffset[panelIndex] + i) * (2.0 * Math.PI)
					/ length;
			double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0)
					/ 2.0;
			if (Double.isNaN(ratio)) {
				ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent)
						+ 1.0) / 2.0;
			}
			if (Double.isNaN(ratio)) {
				ratio = 0.5;
			}
			int pixelIndex = getLedIndex(i);
			int red = (int) (((color[panelIndex][0] * (1 - ratio))
					+ (altColor[panelIndex][0] * ratio)) * brightness[panelIndex]);
			int green = (int) (((color[panelIndex][1] * (1 - ratio))
					+ (altColor[panelIndex][1] * ratio)) * brightness[panelIndex]);
			int blue = (int) (((color[panelIndex][2] * (1 - ratio))
					+ (altColor[panelIndex][2] * ratio)) * brightness[panelIndex]);
			ledBuffer.setRGB(pixelIndex, (int) (red * brightness[panelIndex]),
					(int) (green * brightness[panelIndex]),
					(int) (blue * brightness[panelIndex]));
		}
		wavePhaseOffset[panelIndex] += phaseInc;
		wavePhaseOffset[panelIndex] %= length;
	}

	/**
	 * Sets the LEDs to a breathing effect INTERNAL ONLY
	 */
	private void setBreathing(int panelIndex) {
		breathingPhaseOffset[panelIndex] += (2 * Math.PI * .02)
				/ (flashRateMs[panelIndex] / 1000.0);
		breathingPhaseOffset[panelIndex] %= 2 * Math.PI;
		double intensity = (Math.sin(breathingPhaseOffset[panelIndex]) + 1) / 2;
		// Set each LED to the current color with the calculated intensity
		double startVal = getStartVal(panelIndex);
		double endVal = getEndVal(panelIndex);
		for (int i = (int) startVal; i < endVal; i++) {
			int pixelIndex = getLedIndex(i);
			int red = (int) (color[panelIndex][0] * intensity);
			int green = (int) (color[panelIndex][1] * intensity);
			int blue = (int) (color[panelIndex][2] * intensity);
			ledBuffer.setRGB(pixelIndex, (int) (red * brightness[panelIndex]),
					(int) (green * brightness[panelIndex]),
					(int) (blue * brightness[panelIndex]));
		}
	}

	/**
	 * Sets the LEDs to a GIF INTERNAL ONLY
	 */
	private void setGif(int panelIndex) {
		if (!gifFound(currentImageState[panelIndex])) {
			System.err.println("No images found for ID "
					+ currentImageState[panelIndex].ordinal());
			updateState(new LEDState(LEDStates.OFF, panelIndex));
			return;
		}
		if (currentTimeMs
				- lastUpdateTimeMs[panelIndex] >= flashRateMs[panelIndex]) { //20 +99999999 >= 5000
			byte[][] currentLedStates = LEDConstants.imageLedStates
					.get(currentImageState[panelIndex].ordinal())
					.get(currentImageIndex[panelIndex]);
			double startVal = getStartVal(panelIndex);
			double endVal = getEndVal(panelIndex);
			for (int i = (int) startVal; i < endVal; i++) {
				//Convert i into pixel index, since it's wired up differently
				int pixelIndex = getLedIndex(i);
				ledBuffer.setRGB(pixelIndex,
						(int) (Byte.toUnsignedInt(currentLedStates[i
								- ((int) LEDConstants.ledBufferCutoff * panelIndex)][0])
								* brightness[panelIndex]),
						(int) (Byte.toUnsignedInt(currentLedStates[i
								- ((int) LEDConstants.ledBufferCutoff * panelIndex)][1])
								* brightness[panelIndex]),
						(int) (Byte.toUnsignedInt(currentLedStates[i
								- ((int) LEDConstants.ledBufferCutoff * panelIndex)][2])
								* brightness[panelIndex]));
			}
			currentImageIndex[panelIndex] = (currentImageIndex[panelIndex] + 1)
					% LEDConstants.imageLedStates
							.get(currentImageState[panelIndex].ordinal()).size();
			lastUpdateTimeMs[panelIndex] = currentTimeMs;
		}
	}

	private void setFire(int panelIndex) {
		if (currentTimeMs
				- lastUpdateTimeMs[panelIndex] >= flashRateMs[panelIndex]) {
			// Step 1. Cool down every cell a little
			double startVal = getStartVal(panelIndex);
			double endVal = getEndVal(panelIndex);
			double length = getLength(panelIndex);
			for (int i = (int) startVal; i < endVal; i++) {
				heat[i] = Math.max(0,
						heat[i] - (int) (CommonMath.random.nextDouble()
								* ((COOLING * 10) / length) + 2));
			}
			// Step 2. Heat from each cell drifts 'up' and diffuses a little
			for (int k = (int) (endVal - 1); k >= 5; k--) {
				heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 3] + heat[k - 4]
						+ heat[k - 5]) / 5;
			}
			// Step 3. Randomly ignite new 'sparks' of heat near the bottom
			if (CommonMath.random.nextDouble() * 256 < SPARKING) {
				int y = (int) (CommonMath.random.nextDouble() * 7);
				heat[y] = Math.min(255,
						heat[y] + (int) (CommonMath.random.nextDouble() * 96 + 160));
			}
			// Step 4. Map from heat cells to LED colors
			for (int j = (int) startVal; j < endVal; j++) {
				Color color = heatColor(heat[j]);
				int pixelNumber;
				if (reverseDirection) {
					pixelNumber = (int) (endVal - 1) - j;
				} else {
					pixelNumber = j;
				}
				pixelNumber = getLedIndex(pixelNumber);
				ledBuffer.setRGB(pixelNumber,
						(int) (color.red * 255 * brightness[panelIndex]),
						(int) (color.green * 255 * brightness[panelIndex]),
						(int) (color.blue * 255 * brightness[panelIndex]));
			}
			lastUpdateTimeMs[panelIndex] = currentTimeMs;
		}
	}

	private Color heatColor(int temperature) {
		// Approximate black-body radiation colors for LEDs
		int t192 = (temperature * 192) / 255; // Scale 'heat' down from 0-255 to 0-191
		int heatRamp = t192 & 0x3F; // 0..63
		heatRamp <<= 2; // scale up to 0..252
		if (t192 > 0x80) { // hottest
			return new Color(255, 255, heatRamp);
		} else if (t192 > 0x40) { // middle
			return new Color(255, heatRamp, 0);
		} else { // coolest
			return new Color(heatRamp, 0, 0);
		}
	}

	/**
	 * @param panelIndex
	 */
	private void setText(int panelIndex) {
		String textToDisplay = text[panelIndex];
		textLengthInPixels = textToDisplay.length()
				* (ORIGINAL_CHAR_WIDTH + SPACING) + 1 * SPACING;
		BufferedImage textImage = renderTextToImage(textToDisplay,
				(int) LEDConstants.ledCols, (int) LEDConstants.ledRows, panelIndex,
				offsetX[panelIndex]);
		double startVal = 0, endVal = LEDConstants.ledBufferLength;
		if (LEDConstants.ledBufferCutoff != LEDConstants.ledBufferLength) {
			startVal = getStartVal(panelIndex);
			endVal = getEndVal(panelIndex);
		}
		double numUpdatesInFlashRate = flashRateMs[panelIndex] / 20;
		double increment = textLengthInPixels / numUpdatesInFlashRate;
		// Increment the offsetX
		offsetX[panelIndex] += increment;
		offsetX[panelIndex] %= textLengthInPixels;
		for (int y = 0; y < (int) LEDConstants.ledRows; y++) {
			for (int x = 0; x < (int) LEDConstants.ledCols; x++) {
				int ledIndex = getLedIndex(x, y);
				if (ledIndex >= startVal && ledIndex < endVal) {
					int pixelColor = textImage.getRGB(x, y);
					int red = (pixelColor >> 16) & 0xFF;
					int green = (pixelColor >> 8) & 0xFF;
					int blue = pixelColor & 0xFF;
					ledBuffer.setRGB(ledIndex, red, green, blue);
				}
			}
		}
	}

	private int getLedIndex(int x, int y) {
		if (Constants.currentMode == Mode.SIM) {
			return y * (int) LEDConstants.ledCols + x;
		}
		int index = 0, wrappedX = x;
		// If top right, X's are reversed
		if (LEDConstants.panelOrientation == PanelOrientation.TOP_RIGHT || LEDConstants.panelOrientation == PanelOrientation.BOTTOM_RIGHT) {
			wrappedX = (int) LEDConstants.ledCols - 1 - x;
		}
		if (x >= LEDConstants.ledColsPerPanel) {
			index += LEDConstants.ledColsPerPanel
					* (int) LEDConstants.ledRowsPerPanel;
			wrappedX %= (int) LEDConstants.ledColsPerPanel;
		}
		int adjustedY = y;
		// If bottom right / left, Y's are reversed
		if (LEDConstants.panelOrientation == PanelOrientation.BOTTOM_RIGHT || LEDConstants.panelOrientation == PanelOrientation.BOTTOM_LEFT) {
			adjustedY = (int) LEDConstants.ledRowsPerPanel - 1 - y;
		}
		// Determine if the row is reversed (serpentine)
		if (adjustedY % 2 == 1) {
			// Reverse the x coordinate for odd rows
			wrappedX = (int) LEDConstants.ledColsPerPanel - 1 - wrappedX;
		}
		// Calculate the index
		index += adjustedY * LEDConstants.ledColsPerPanel + wrappedX;

		return index;
	}

	private int getLedIndex(int index) {
		if (Constants.currentMode == Mode.SIM
				|| LEDConstants.ledBufferCutoff <= index) {
			return index;
		}
		int translatedY = index / (int) LEDConstants.ledCols;
		int translatedX = index % (int) LEDConstants.ledCols;
		return getLedIndex(translatedX, translatedY);
	}

	double textLengthInPixels = 7;
	private static final int ORIGINAL_CHAR_WIDTH = 8;
	private static final int ORIGINAL_CHAR_HEIGHT = 8;
	private static final int SPACING = 1;

	public BufferedImage renderTextToImage(String text, int width, int height,
			int panelIndex, double offsetX) {
		BufferedImage image = new BufferedImage(width, height,
				BufferedImage.TYPE_INT_ARGB);
		Graphics2D g = image.createGraphics();
		// Set the text color
		g.setColor(new java.awt.Color(color[panelIndex][0], color[panelIndex][1],
				color[panelIndex][2]));
		// Calculate starting position to center the text
		//int startX = (width - ((charWidth + spacing) * text.length() - spacing)) / 2;
		int startY = (height - ORIGINAL_CHAR_HEIGHT) / 2;
		// Render each character in the text
		int x = (int) -offsetX;
		for (char c : text.toCharArray()) {
			drawCharacter(g, c, x, startY);
			x += ORIGINAL_CHAR_WIDTH + SPACING; // Move to the next character position
		}
		g.dispose();
		return image;
	}

	private static void drawCharacter(Graphics2D g, char c, int x, int y) {
		// Get the character bitmap
		boolean[][] bitmap = CHAR_MAP.getOrDefault(Character.toUpperCase(c),
				CHAR_MAP.get('A')); // Default to 'A' if unknown
		// Draw the bitmap pixel by pixel, scaling to the new character width and height
		for (int row = 0; row < ORIGINAL_CHAR_HEIGHT; row++) {
			for (int col = 0; col < ORIGINAL_CHAR_WIDTH; col++) {
				if (bitmap[row][col]) {
					g.fillRect(x + col, y + row, 1, 1); // Draw a single pixel
				}
			}
		}
	}

	/**
	 * Resets all time specific variables
	 */
	private void resetTimeSpecificVariables(int panelIndex) {
		lastUpdateTimeMs[panelIndex] = Double.NEGATIVE_INFINITY;
	}

	/**
	 * Updates all functions to the new flash rate "FlashRate" is defined as the
	 * either the ENTIRE time for a single cycle of the LED state, or, for a gif,
	 * the time between each frame
	 * 
	 * @param flashRateMilliSeconds
	 */
	public void updateFlashRate(int flashRateMilliSeconds, int panelIndex) {
		this.flashRateMs[panelIndex] = flashRateMilliSeconds;
		//resetTimeSpecificVariables();
	}

	/**
	 * Updates all functions to a new brightness
	 * 
	 * @param brightness (0-1)
	 */
	public void updateBrightness(int brightness, int panelIndex) {
		this.brightness[panelIndex] = brightness;
	}

	/**
	 * Updates all functions to a new color
	 * 
	 * @param color
	 */
	public void updateColor(int[] color, int panelIndex) {
		this.color[panelIndex] = color;
	}

	/**
	 * Updates all functions to a new alternate color
	 * 
	 * @param altColor (Used in wave2)
	 */
	public void updateAltColor(int[] altColor, int panelIndex) {
		this.altColor[panelIndex] = altColor;
	}

	/**
	 * Updates all functions to a new state
	 * 
	 * @param state
	 */
	public void updateState(LEDState state) {
		if (state.state == LEDStates.OFF) {
			timeCheck = 0;
			/*             ^Set Checked state^
			 * Example of time checking (Used when debugging if a pattern is completing at the correct flashRate)
			 * IN METHOD OF PATTERN:
			 * if (timeCheck >= FULL_CYCLE_OF_PATTERN) { 
			 * 	throw new IllegalArgumentException("Pattern completed after " + timeCheck + 
			 * 	"ms when the desired flash rate was " + flashRateMs[panelIndex] + " ms."); 
			 * }
			 */
		}
		if (state.state == LEDStates.GIF) {
			currentImageIndex[state.panelIndex] = 0;
			resetTimeSpecificVariables(state.panelIndex);
		} else if (state.state == LEDStates.FIRE) {
			resetTimeSpecificVariables(state.panelIndex);
		}
		currentLEDState[state.panelIndex] = state.state;
	}

	/**
	 * Updates all functions (really just Gif) to a new image state
	 * 
	 * @param imageState
	 */
	public void updateImageState(ImageStates imageState, int panelIndex) {
		currentImageState[panelIndex] = imageState;
	}

	/**
	 * Updates all functions to a new state, with additional arguments for the
	 * flash rate, color, and image state.
	 * 
	 * @param state The new LED state with it's panel index.
	 * @param args  Additional arguments that can be:
	 *                 <ul>
	 *                 <li>An int array representing the color. The array should
	 *                 contain three elements representing the RGB values. Each
	 *                 value should be between 0 and 255. Check
	 *                 {@link LEDConstants} or
	 *                 {@link edu.wpi.first.wpilibj.util.Color} for default
	 *                 colors (For WPILib.color, mulitiply by 255 for RGB
	 *                 vals)</li>
	 *                 <li>An int array representing the alternate color. The
	 *                 array should contain three elements representing the RGB
	 *                 values. Each value should be between 0 and 255. Check
	 *                 {@link LEDConstants} or
	 *                 {@link edu.wpi.first.wpilibj.util.Color} for default
	 *                 colors (For WPILib.color, mulitiply by 255 for RGB
	 *                 vals)</li>
	 *                 <li>An integer representing the flash rate in
	 *                 milliseconds.</li>
	 *                 <li>An ImageState object representing the image
	 *                 state.</li>
	 *                 </ul>
	 *                 The order of these arguments does not matter.
	 */
	public void updateLEDState(LEDState state, Object... args) {
		if (state.state != currentLEDState[state.panelIndex])
			updateState(state);
		boolean hasColor = false, hasText = false;
		for (Object arg : args) {
			if (arg instanceof int[]) {
				if (!hasColor) {
					updateColor((int[]) arg, state.panelIndex);
					hasColor = true;
				} else {
					updateAltColor((int[]) arg, state.panelIndex);
				}
			} else if (arg instanceof Double || arg instanceof Integer) {
				updateFlashRate((int) arg, state.panelIndex);
			} else if (arg instanceof ImageStates) {
				updateImageState((ImageStates) arg, state.panelIndex);
			} else if (arg instanceof String) {
				if (!hasText) {
					System.out.println("Text: " + arg);
					text[state.panelIndex] = (String) arg;
					offsetX[state.panelIndex] = 0;
					//reset 
					hasText = true;
				} else {
					fontString[state.panelIndex] = (String) arg;
				}
			} else {
				throw new IllegalArgumentException("Unexpected argument type: "
						+ arg.getClass().getSimpleName());
			}
		}
	}

	public static List<List<byte[][]>> preprocessImages(List<String> gifPaths) {
		List<List<byte[][]>> imageList = new ArrayList<>();
		for (int i = 0; i < gifPaths.size(); i++) {
			List<byte[][]> gifImages = new ArrayList<>();
			String gifPath;
			if (Constants.currentMode == Mode.REAL) {
				gifPath = gifStorage + "/" + gifPaths.get(i);
			} else {
				gifPath = gifStorage + "\\" + gifPaths.get(i);
			}
			List<String> filePaths = new ArrayList<>();
			try {
				Files.walkFileTree(Paths.get(gifPath),
						new SimpleFileVisitor<Path>() {
							@Override
							public FileVisitResult visitFile(Path file,
									BasicFileAttributes attrs) throws IOException {
								filePaths.add(file.toString());
								System.err.println(file.toString());
								return FileVisitResult.CONTINUE;
							}
						});
			}
			catch (IOException e) {
				e.printStackTrace();
			}
			for (String path : filePaths) {
				try {
					BufferedImage image = ImageIO.read(new File(path));
					gifImages.add(processImageToLedStates(image));
				}
				catch (IOException e) {
					e.printStackTrace();
				}
			}
			imageList.add(gifImages);
		}
		return imageList;
	}

	public boolean gifFound(ImageStates imageState) {
		return !LEDConstants.imageLedStates.get(imageState.ordinal()).isEmpty();
	}

	private static byte[][] processImageToLedStates(BufferedImage image) {
		int ledCount = (int) LEDConstants.ledRows * (int) LEDConstants.ledCols;
		// Resize image to fit the number of LEDs
		BufferedImage resizedImage = new BufferedImage((int) LEDConstants.ledCols,
				(int) LEDConstants.ledRows, BufferedImage.TYPE_INT_RGB);
		resizedImage.getGraphics().drawImage(image, 0, 0,
				(int) LEDConstants.ledCols, (int) LEDConstants.ledRows, null);
		// Initialize ledStates from the image pixels
		byte[][] ledStates = new byte[ledCount][3];
		for (int y = 0; y < LEDConstants.ledRows; y++) {
			for (int x = 0; x < LEDConstants.ledCols; x++) {
				int pixel = resizedImage.getRGB(x, y);
				byte red = (byte) ((pixel >> 16) & 0xFF);
				byte green = (byte) ((pixel >> 8) & 0xFF);
				byte blue = (byte) (pixel & 0xFF);
				ledStates[x + y * (int) LEDConstants.ledCols][0] = red;
				ledStates[x + y * (int) LEDConstants.ledCols][1] = green;
				ledStates[x + y * (int) LEDConstants.ledCols][2] = blue;
			}
		}
		return ledStates;
	}

	/**
	 * Does NOT self check. User must look at the images.
	 */
	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(
				run(() -> updateLEDState(new LEDState(LEDStates.RAINBOW, 0)))
						.withTimeout(5),
				runOnce(() -> {
					System.out.println("OVER");
				})).until(() -> !getFaults().isEmpty());
	}

	@Override
	public double getCurrent() { return 0; }

	@Override
	public HashMap<String, Double> getTemps() {
		return new HashMap<String, Double>(Map.of("NULL", 0.0));
	}

	@Override
	public void setCurrentLimit(int amps) { return; }

    public void runHandler() { 
		//Handle the LED logic here
		if (Constants.currentMatchState == FRCMatchState.DISABLED) {
			updateLEDState(new LEDState(LEDStates.SOLID_COLOR,0), LEDConstants.disabledRGB);
		} else if (Constants.currentMatchState == FRCMatchState.AUTO) {
			updateLEDState(new LEDState(LEDStates.RAINBOW,0), 1000);
		} else if (Constants.currentMatchState == FRCMatchState.TELEOP) {
			updateLEDState(new LEDState(LEDStates.FIRE,0), 20);
		} else if (Constants.currentMatchState == FRCMatchState.TEST) {
			updateLEDState(new LEDState(LEDStates.GIF,0), 40, ImageStates.gif1);
		}
	 }
}

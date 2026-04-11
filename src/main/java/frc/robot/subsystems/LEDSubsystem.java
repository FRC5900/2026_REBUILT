package frc.robot.subsystems;

import java.util.Random;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

  // ── Pattern catalogue ─────────────────────────────────────────────────────────
  public enum Pattern {
    // ── Animated ──────────────────────────────────────────────────────────────
    RAINBOW,           //  0
    RAINBOW_GLITTER,   //  1
    CONFETTI,          //  2
    SHOT_RED,          //  3
    SHOT_BLUE,         //  4
    SHOT_WHITE,        //  5
    SINELON,           //  6
    BPM,               //  7
    FIRE,              //  8
    TWINKLES,          //  9
    COLOR_WAVES,       // 10
    LARSON_RED,        // 11
    LARSON_GRAY,       // 12
    CHASE_RED,         // 13
    CHASE_BLUE,        // 14
    CHASE_GRAY,        // 15
    HEARTBEAT_RED,     // 16
    HEARTBEAT_BLUE,    // 17
    HEARTBEAT_WHITE,   // 18
    BREATH_RED,        // 19
    BREATH_BLUE,       // 20
    BREATH_GRAY,       // 21
    STROBE_RED,        // 22
    STROBE_BLUE,       // 23
    STROBE_GOLD,       // 24
    STROBE_WHITE,      // 25
    GRADIENT_LOOP,     // 26 pink → yellow → blue
    TEAM_COLORS,       // 27 — white comets on navy
    // ── Solid colours  ────────────────────────────────
    HOT_PINK,          // 28
    DARK_RED,          // 29
    RED,               // 30
    RED_ORANGE,        // 31
    ORANGE,            // 32
    GOLD,              // 33
    YELLOW,            // 34
    LAWN_GREEN,        // 35
    LIME,              // 36
    DARK_GREEN,        // 37
    GREEN,             // 38
    BLUE_GREEN,        // 39
    AQUA,              // 40
    SKY_BLUE,          // 41
    DARK_BLUE,         // 42
    BLUE,              // 43
    BLUE_VIOLET,       // 44
    VIOLET,            // 45
    WHITE,             // 46
    GRAY,              // 47
    DARK_GRAY,         // 48
    OFF;               // 49

    private static final Pattern[] VALUES = values();

    public static Pattern fromId(int id) {
      if (id < 0 || id >= VALUES.length) return OFF;
      return VALUES[id];
    }

    public static int count() { return VALUES.length; }

    public static Pattern next(Pattern p) {
      return VALUES[(p.ordinal() + 1) % VALUES.length];
    }
  }

  // ── Solid colour table ───────────
  private static final int HOT_PINK_ORD = Pattern.HOT_PINK.ordinal();
  private static final int[][] SOLID_RGB = {
    {255,  20, 147},  // HOT_PINK
    {139,   0,   0},  // DARK_RED
    {255,   0,   0},  // RED
    {255,  70,   0},  // RED_ORANGE
    {255, 120,   0},  // ORANGE
    {255, 180,   0},  // GOLD
    {255, 220,   0},  // YELLOW
    {124, 252,   0},  // LAWN_GREEN
    { 50, 205,  50},  // LIME
    {  0, 100,   0},  // DARK_GREEN
    {  0, 255,   0},  // GREEN
    {  0, 200, 100},  // BLUE_GREEN
    {  0, 255, 200},  // AQUA
    {100, 180, 255},  // SKY_BLUE
    {  0,   0, 139},  // DARK_BLUE
    {  0,   0, 255},  // BLUE
    {138,  43, 226},  // BLUE_VIOLET
    {148,   0, 211},  // VIOLET
    {255, 255, 255},  // WHITE
    {100, 100, 100},  // GRAY
    { 50,  50,  50},  // DARK_GRAY
    {  0,   0,   0},  // OFF
  };

  private static final int GRADIENT_START_HUE = 165;

  // ── Fields ────────────────────────────────────────────────────────────────────
  private final AddressableLED        m_led;
  private final AddressableLEDBuffer  m_buffer;
  private final Random                m_random = new Random();
  private final int[]                 m_heat;

  private Pattern m_pattern = Pattern.TWINKLES;
  private int     m_tick    = 0;
  private int     m_offset  = 0;

  private int     m_larsonPos = 0;
  private int     m_larsonDir = 1;
  private int     m_shotPos   = 0;
  private boolean m_prevDSAttached = false;

  // Debug cycling
  private boolean m_debugMode  = false;
  private static final int DEBUG_TICKS = 500;

  private static final int SCROLL_SPEED = 3; // pixels per tick for "GRADIENT_LOOP"

  // ── Constructor ───────────────────────────────────────────────────────────────
  public LEDSubsystem() {
    m_led    = new AddressableLED(LEDConstants.kLEDPort);
    m_buffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
    m_heat   = new int[LEDConstants.kLEDLength];
    m_led.setLength(LEDConstants.kLEDLength);

  
    int len = m_buffer.getLength();
    int sw  = 18; // stripe width
    for (int i = 0; i < len; i++) {
      if ((i / sw) % 2 == 0) m_buffer.setRGB(i, 255, 255, 255);
      else                    m_buffer.setRGB(i, 0, 8, 45);
    }
    m_led.setData(m_buffer);
    m_led.start();
  }


  public void setPattern(Pattern pattern) {
    if (m_pattern != pattern) {
      // Reset animation state when changing patterns
      m_offset    = 0;
      m_larsonPos = 0;
      m_larsonDir = 1;
      m_shotPos   = 0;
    }
    m_pattern = pattern;
  }

  public void setPattern(int id) {
    setPattern(Pattern.fromId(id));
  }

  // enable debug mode; only is changed in robotcontainer
  public void setDebugMode(boolean enabled) {
    m_debugMode = enabled;
  }

  public Pattern getCurrentPattern() { return m_pattern; }

  // ── Periodic ──────────────────────────────────────────────────────────────────
  @Override
  public void periodic() {
    boolean dsNow = DriverStation.isDSAttached();
    if (dsNow && !m_prevDSAttached) {
      fill(0, 0, 0);
    }
    m_prevDSAttached = dsNow;

    if (!dsNow) {
      stripeWhiteNavy();
    } else if (DriverStation.isAutonomousEnabled()) {
      twinkles();
    } else {
      if (m_debugMode) {
        SmartDashboard.putNumber("LED Pattern ID",   m_pattern.ordinal());
        SmartDashboard.putString("LED Pattern Name", m_pattern.name());
        if (m_tick > 0 && m_tick % DEBUG_TICKS == 0) {
          m_pattern   = Pattern.next(m_pattern);
          m_offset    = 0;
          m_larsonPos = 0;
          m_larsonDir = 1;
          m_shotPos   = 0;
        }
      }
      renderPattern();
    }

    // Solid red on the last 5 LEDs — applied after every pattern, single setData below.
    redIndicator();
    m_led.setData(m_buffer);
    m_tick++;
  }

  // ── Render patterns ─────────────────────────────────────────────────────────
  private void renderPattern() {
    switch (m_pattern) {
      case RAINBOW         -> rainbow(false);
      case RAINBOW_GLITTER -> rainbow(true);
      case CONFETTI        -> confetti();
      case SHOT_RED        -> shot(255,   0,   0);
      case SHOT_BLUE       -> shot(  0,   0, 255);
      case SHOT_WHITE      -> shot(255, 255, 255);
      case SINELON         -> sinelon();
      case BPM             -> bpm();
      case FIRE            -> fire();
      case TWINKLES        -> twinkles();
      case COLOR_WAVES     -> colorWaves();
      case LARSON_RED      -> larsonScanner(255,   0,   0);
      case LARSON_GRAY     -> larsonScanner(150, 150, 150);
      case CHASE_RED       -> lightChase(255,   0,   0);
      case CHASE_BLUE      -> lightChase(  0,   0, 255);
      case CHASE_GRAY      -> lightChase(150, 150, 150);
      case HEARTBEAT_RED   -> heartbeat(255,   0,   0);
      case HEARTBEAT_BLUE  -> heartbeat(  0,   0, 255);
      case HEARTBEAT_WHITE -> heartbeat(255, 255, 255);
      case BREATH_RED      -> breath(255,   0,   0);
      case BREATH_BLUE     -> breath(  0,   0, 255);
      case BREATH_GRAY     -> breath(150, 150, 150);
      case STROBE_RED      -> strobe(255,   0,   0);
      case STROBE_BLUE     -> strobe(  0,   0, 255);
      case STROBE_GOLD     -> strobe(255, 180,   0);
      case STROBE_WHITE    -> strobe(255, 255, 255);
      case GRADIENT_LOOP   -> gradientLoop();
      case TEAM_COLORS     -> teamColors();
      default              -> solidColor();
    }
  }

  // ── Animated patterns ─────────────────────────────────────────────────────────

  // Full rainbow scrolling pattern across the strip with white glitter
  private void rainbow(boolean glitter) {
    int len = m_buffer.getLength();
    for (int i = 0; i < len; i++) {
      int hue = (int)((i * 180.0 / len + m_tick * 2) % 180);
      m_buffer.setHSV(i, hue, 255, 128);
    }
    if (glitter && m_random.nextInt(10) == 0) {
      m_buffer.setRGB(m_random.nextInt(len), 255, 255, 255);
    }
  }

  // Random coloured sparkles on a fading black background
  private void confetti() {
    int len = m_buffer.getLength();
    for (int i = 0; i < len; i++) {
      var c = m_buffer.getLED8Bit(i);
      m_buffer.setRGB(i,
          Math.max(0, c.red   - 12),
          Math.max(0, c.green - 12),
          Math.max(0, c.blue  - 12));
    }
    if (m_random.nextInt(3) == 0) {
      m_buffer.setHSV(m_random.nextInt(len), m_random.nextInt(180), 200, 255);
    }
  }

  // Single coloured dot going around the strip with a fading trail
  private void shot(int r, int g, int b) {
    int len = m_buffer.getLength();
    fill(0, 0, 0);
    int trailLen = 14;
    for (int t = 0; t < trailLen; t++) {
      int pos = (m_shotPos - t + len) % len;
      double br = 1.0 - t / (double) trailLen;
      m_buffer.setRGB(pos, (int)(r * br), (int)(g * br), (int)(b * br));
    }
    m_shotPos = (m_shotPos + 4) % len;
  }

  // Dot moving back and forth with a trail
  private void sinelon() {
    int len = m_buffer.getLength();
    for (int i = 0; i < len; i++) {
      var c = m_buffer.getLED8Bit(i);
      m_buffer.setRGB(i,
          Math.max(0, c.red   - 15),
          Math.max(0, c.green - 15),
          Math.max(0, c.blue  - 15));
    }
    int pos = (int)((Math.sin(m_tick * 0.05) * 0.5 + 0.5) * (len - 1));
    m_buffer.setHSV(pos, m_tick % 180, 255, 255);
  }

  // Rainbow hue across the strip
  private void bpm() {
    int len = m_buffer.getLength();
    double brightness = Math.sin(m_tick * 2 * Math.PI / 25.0) * 0.5 + 0.5;
    int v = (int)(brightness * 255);
    for (int i = 0; i < len; i++) {
      int hue = (int)(i * 180.0 / len + m_tick) % 180;
      m_buffer.setHSV(i, hue, 255, v);
    }
  }

  // fire
  private void fire() {
    int len = m_heat.length;
    for (int i = 0; i < len; i++) {
      m_heat[i] = Math.max(0, m_heat[i] - m_random.nextInt(6));
    }
    for (int i = len - 1; i > 2; i--) {
      m_heat[i] = (m_heat[i-1] + m_heat[i-2] + m_heat[i-3]) / 3;
    }
    if (m_random.nextInt(3) == 0) {
      int y = m_random.nextInt(8);
      m_heat[y] = Math.min(255, m_heat[y] + m_random.nextInt(180) + 50);
    }
    for (int i = 0; i < len; i++) {
      int t192 = m_heat[i] * 3 / 4;
      int ramp  = (t192 & 63) << 2;
      if      (t192 > 128) m_buffer.setRGB(i, 255, 255, ramp);
      else if (t192 >  64) m_buffer.setRGB(i, 255, ramp,   0);
      else                 m_buffer.setRGB(i, ramp,   0,   0);
    }
  }

  // Random sparkle in random colours then fade
  private void twinkles() {
    int len = m_buffer.getLength();
    for (int i = 0; i < len; i++) {
      var c = m_buffer.getLED8Bit(i);
      m_buffer.setRGB(i,
          Math.max(0, c.red   - 6),
          Math.max(0, c.green - 6),
          Math.max(0, c.blue  - 6));
    }
    for (int j = 0; j < 5; j++) {
      if (m_random.nextInt(4) == 0) {
        m_buffer.setHSV(m_random.nextInt(len), m_random.nextInt(180), 200, 200);
      }
    }
  }

  // colour sweep
  private void colorWaves() {
    int len = m_buffer.getLength();
    for (int i = 0; i < len; i++) {
      double phase = (i * Math.PI * 4.0 / len) + m_tick * 0.04;
      int hue = (int)((Math.sin(phase) * 0.5 + 0.5) * 180);
      m_buffer.setHSV(i, hue, 255, 200);
    }
  }

  // bouncing dot
  private void larsonScanner(int r, int g, int b) {
    int len = m_buffer.getLength();
    for (int i = 0; i < len; i++) {
      var c = m_buffer.getLED8Bit(i);
      m_buffer.setRGB(i,
          Math.max(0, c.red   - 25),
          Math.max(0, c.green - 25),
          Math.max(0, c.blue  - 25));
    }
    int tailLen = 10;
    for (int t = 0; t < tailLen; t++) {
      int pos = m_larsonPos - t * m_larsonDir;
      if (pos >= 0 && pos < len) {
        double br = 1.0 - t / (double) tailLen;
        m_buffer.setRGB(pos, (int)(r * br), (int)(g * br), (int)(b * br));
      }
    }
    if (m_tick % 2 == 0) {
      m_larsonPos += m_larsonDir;
      if (m_larsonPos >= len - 1 || m_larsonPos <= 0) m_larsonDir *= -1;
    }
  }

  // Groups of dots chasing
  private void lightChase(int r, int g, int b) {
    int len = m_buffer.getLength();
    fill(0, 0, 0);
    int spacing = 10;
    for (int i = 0; i < len; i += spacing) {
      int pos = (i + m_offset) % len;
      m_buffer.setRGB(pos, r, g, b);
      if (pos + 1 < len) m_buffer.setRGB(pos + 1, r / 2, g / 2, b / 2);
    }
    if (m_tick % 2 == 0) m_offset = (m_offset + 1) % len;
  }

  // heartbeat
  private void heartbeat(int r, int g, int b) {
    int phase = m_tick % 100;
    double brightness = 0;
    if      (phase < 15) brightness = Math.sin(phase * Math.PI / 15.0);
    else if (phase < 30) brightness = Math.sin((phase - 15) * Math.PI / 15.0) * 0.6;
    brightness = Math.max(0, brightness);
    fill((int)(r * brightness), (int)(g * brightness), (int)(b * brightness));
  }

  // breathe white
  private void breath(int r, int g, int b) {
    double brightness = Math.sin(m_tick * 2 * Math.PI / 150.0) * 0.5 + 0.5;
    fill((int)(r * brightness), (int)(g * brightness), (int)(b * brightness));
  }

  // strobe white
  private void strobe(int r, int g, int b) {
    boolean on = (m_tick % 8) == 0;
    fill(on ? r : 0, on ? g : 0, on ? b : 0);
  }

// pink yellow blue scrolling
  private void gradientLoop() {
    int len = m_buffer.getLength();
    for (int i = 0; i < len; i++) {
      double t = ((i + m_offset) % len) / (double) len;
      int hue = (int)((GRADIENT_START_HUE + t * 180) % 180);
      m_buffer.setHSV(i, hue, 230, 240);
    }
    m_offset = (m_offset + SCROLL_SPEED) % len;
  }

// 5900 comets
  private void teamColors() {
    int len       = m_buffer.getLength();
    int numComets = 5;
    int spacing   = len / numComets;  // how far apart
    int trailLen  = 80;

    fill(0, 8, 45);

    for (int c = 0; c < numComets; c++) {
      int head = (m_offset + c * spacing) % len;
      for (int t = 0; t < trailLen; t++) {
        int pos = (head - t + len) % len;
        double br = 1.0 - t / (double) trailLen;
        m_buffer.setRGB(pos,
            (int)(br * 255),
            (int)(8  + br * (255 - 8)),
            (int)(45 + br * (255 - 45)));
      }
    }
    m_offset = (m_offset + 2) % len;
  }

  //  Scrolling white-and-navy barber-pole stripes
  private void stripeWhiteNavy() {
    int len = m_buffer.getLength();
    int stripeWidth = 10;           // width of each band
    int period = stripeWidth * 2;
    int scroll = m_tick % period;   // px per tick
    for (int i = 0; i < len; i++) {
      int pos = (i + scroll) % period;
      if (pos < stripeWidth) m_buffer.setRGB(i, 255, 255, 255);
      else                   m_buffer.setRGB(i, 0, 8, 45);
    }
  }

  //blinky red eye
  private void redIndicator() {
    int len = m_buffer.getLength();
    boolean on = (m_tick / 12) % 2 == 0;
    for (int i = len - 5; i < len; i++) {
      m_buffer.setRGB(i, on ? 255 : 0, 0, 0);
    }
  }

  // ── Solid colour renderer ─────────────────────────────────────────────────────
  private void solidColor() {
    int idx = m_pattern.ordinal() - HOT_PINK_ORD;
    int[] c = (idx >= 0 && idx < SOLID_RGB.length) ? SOLID_RGB[idx] : new int[]{0, 0, 0};
    fill(c[0], c[1], c[2]);
  }

  // ── Helpers ───────────────────────────────────────────────────────────────────
  private void fill(int r, int g, int b) {
    for (int i = 0; i < m_buffer.getLength(); i++) m_buffer.setRGB(i, r, g, b);
  }

}

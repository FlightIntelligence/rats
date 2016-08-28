package control.dto;

/**
 * FlipDirection for flipping.
 *
 * @author Hoang Tung Dinh
 * @see <a
 *     href="http://bebop-autonomy.readthedocs.io/en/latest/piloting.html#flight-animations">Bebop
 *     documentations</a>
 */
public enum FlipDirection {
  FORWARD((byte) 0),
  BACKWARD((byte) 1),
  RIGHT((byte) 2),
  LEFT((byte) 3);

  private final byte code;

  FlipDirection(byte code) {
    this.code = code;
  }

  /**
   * Get the code associated with each flipping direction according to bebop documents.
   *
   * @see <a
   *     href="http://bebop-autonomy.readthedocs.io/en/latest/piloting.html#flight-animations">Bebop
   *     ROS driver</a>
   */
  public byte getCode() {
    return code;
  }
}

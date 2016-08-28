package control;

/** @author Hoang Tung Dinh */
public enum DefaultPidParameters {
  LINEAR_X {
    @Override
    public PidParameters getParameters() {
      return PidParameters.builder()
          .setKp(0.6)
          .setKd(0.3)
          .setKi(0)
          .setLagTimeInSeconds(0.2)
          .build();
    }
  },

  LINEAR_Y {
    @Override
    public PidParameters getParameters() {
      return PidParameters.builder()
          .setKp(0.6)
          .setKd(0.3)
          .setKi(0)
          .setLagTimeInSeconds(0.2)
          .build();
    }
  },

  LINEAR_Z {
    @Override
    public PidParameters getParameters() {
      return PidParameters.builder()
          .setKp(0.6)
          .setKd(0.3)
          .setKi(0)
          .setLagTimeInSeconds(0.2)
          .build();
    }
  },

  ANGULAR_Z {
    @Override
    public PidParameters getParameters() {
      return PidParameters.builder()
          .setKp(1.5)
          .setKd(0.75)
          .setKi(0)
          .setLagTimeInSeconds(0.2)
          .build();
    }
  };

  public abstract PidParameters getParameters();
}

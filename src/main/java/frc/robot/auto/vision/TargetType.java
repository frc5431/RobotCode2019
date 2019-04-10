package frc.robot.auto.vision;

public enum TargetType{
    FRONT_RIGHT(Limelight.FRONT, 0), FRONT_LEFT(Limelight.FRONT, 1);

    private final Limelight llight;
    private final int pipeline;

    private TargetType(final Limelight light, final int pipe){
        this.llight = light;
        this.pipeline = pipe;
    }

    public Limelight getLimelight(){
        return llight;
    }

    public int getPipeline(){
        return pipeline;
    }
};
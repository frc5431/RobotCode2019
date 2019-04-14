package frc.robot.auto.vision;

public enum TargetType{
    FRONT_RIGHT(0), FRONT_LEFT(1);

    private final int pipeline;

    private TargetType(final int pipe){
        this.pipeline = pipe;
    }

    public int getPipeline(){
        return pipeline;
    }
};
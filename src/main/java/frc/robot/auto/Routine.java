package frc.robot.auto;

public enum Routine{
    HAB2_TO_FROCKET_2_RIGHT(Sequence.ROCKET_FORWARD_2, Path.HAB2_TO_FROCKET_GEN, null, null, null),
    HAB2_TO_FROCKET_2_LEFT(Sequence.ROCKET_FORWARD_2, Path.HAB2_TO_FROCKET_GEN, null, null, null, true),
    CARGOSHIP_FRONT_LEFT(Sequence.ROCKET_FORWARD_1, Path.HAB_TO_CCARGO_GEN, null, null),
    CARGOSHIP_FRONT_RIGHT(Sequence.ROCKET_FORWARD_1, Path.HAB_TO_CCARGO_GEN, null, null, true),
    CARGOSHIP_SIDE_RIGHT(Sequence.ROCKET_FORWARD_1, Path.HAB2_TO_SCARGO_GEN, Path.SCARGO_TO_LS_GEN, Path.LS_TO_SCARGO_GEN),
    DO_NOTHING(null, null, null, null);

    private final Path startHatch, loadingStation, secondHatch;
    private final Sequence firstSequence, secondSequence;
    private final boolean swapped;

    private Routine(final Sequence sequence, final Path startHatch, final Path loadingStation, final Path secondHatch){
        this(sequence, startHatch, loadingStation, secondHatch, false);
    }

    private Routine(final Sequence sequence, final Path startHatch, final Path loadingStation, final Path secondHatch, final boolean swapped){
        this(sequence, startHatch, loadingStation, secondHatch, sequence, swapped);
    }

    private Routine(final Sequence firstSequence, final Path startHatch, final Path loadingStation, final Path secondHatch, final Sequence secondSequence){
        this(firstSequence, startHatch, loadingStation, secondHatch, secondSequence, false);
    }

    private Routine(final Sequence firstSequence, final Path startHatch, final Path loadingStation, final Path secondHatch, final Sequence secondSequence, final boolean swapped){
        this.firstSequence = firstSequence;
        this.secondSequence = secondSequence;
        this.startHatch = startHatch;
        this.loadingStation = loadingStation;
        this.secondHatch = secondHatch;
        this.swapped = swapped;
    }

    public Sequence getFirstSequence(){
        return firstSequence;
    }

    public Sequence getSecondSequence(){
        return secondSequence;
    }

    public Path getStartHatchPath(){
        return startHatch;
    }

    public Path getLoadingStationPath(){
        return loadingStation;
    }

    public Path getSecondHatchPath(){
        return secondHatch;
    }

    public boolean isSwapped(){
        return swapped;
    }
};

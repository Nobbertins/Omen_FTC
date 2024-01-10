package org.firstinspires.ftc.teamcode;

//represent the possible locations of a marker either relative or absolute
public enum MarkerPosition {
    UNKNOWN(-1), LEFT(0), MIDDLE(1),  RIGHT(2);

    public final int value;

    private MarkerPosition(int value){
        this.value = value;
    }

}

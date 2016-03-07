package pl.edu.agh.capo.logic.common;

public enum AgentMove {
    UP ('w'),
    DOWN ('s'),
    LEFT ('a'),
    RIGHT ('d'),
    ROTATE_LEFT('q'),
    ROTATE_RIGHT ('e');

    private int moveCode;
    AgentMove(int moveCode) {
        this.moveCode = moveCode;
    }

    private boolean Compare(int i){return moveCode == i;}
    public static AgentMove getValue(int _id)
    {
        AgentMove[] As = AgentMove.values();
        for (AgentMove A : As) {
            if (A.Compare(_id))
                return A;
        }
        return null;
    }
}

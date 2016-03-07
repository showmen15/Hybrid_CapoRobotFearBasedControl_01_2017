package pl.edu.agh.capo.maze;

import java.awt.*;

public class Wall {

    private String id;

    private double width;

    private double height;

    private String color;

    private Coordinates from;

    private Coordinates to;

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public double getWidth() {
        return width;
    }

    public void setWidth(double width) {
        this.width = width;
    }

    public double getHeight() {
        return height;
    }

    public void setHeight(double height) {
        this.height = height;
    }

    public Color getColor() {
        return Color.decode(color);
    }

    public void setColor(String color) {
        this.color = color;
    }

    public Coordinates getFrom() {
        return from;
    }

    public void setFrom(Coordinates from) {
        this.from = from;
    }

    public Coordinates getTo() {
        return to;
    }

    public void setTo(Coordinates to) {
        this.to = to;
    }
}

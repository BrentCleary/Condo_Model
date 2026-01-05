using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Room : MonoBehaviour
{
    public string roomName;
    public Vector2Int gridPosition;
    public Vector2Int size; // Size in grid units
    public List<Door> doors = new List<Door>();

    private void Awake()
    {
        // Initialize doors by finding Door components in children
        doors.AddRange(GetComponentsInChildren<Door>());
    }

    public Vector2Int GetBottomLeftGridPosition()
    {
        return gridPosition;
    }

    public Vector2Int GetTopRightGridPosition()
    {
        return new Vector2Int(gridPosition.x + size.x - 1, gridPosition.y + size.y - 1);
    }

    public bool ContainsGridPosition(Vector2Int position)
    {
        return position.x >= gridPosition.x && position.x < gridPosition.x + size.x &&
        position.y >= gridPosition.y && position.y < gridPosition.y + size.y;
    }
}

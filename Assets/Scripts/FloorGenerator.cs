using System;
using System.Collections.Generic;
using UnityEngine;

public class FloorGenerator : MonoBehaviour
{

	// Start is called once before the first execution of Update after the MonoBehaviour is created

	void Start()
	{
		new RoomIndex().Start();          // ← actually populate the list
	
		// Example: generate the Kitchen by looking it up in the list
		RoomIndex.FloorSection livingRoom = RoomIndex.FloorSectionList
				.Find(s => s.Name == "LivingRoom");

		RoomIndex.FloorSection diningRoom = RoomIndex.FloorSectionList
				.Find(s => s.Name == "DiningRoom");

		RoomIndex.FloorSection bedroom		= RoomIndex.FloorSectionList
				.Find(s => s.Name == "Bedroom");

		if (livingRoom != null)
			GenerateFloor(livingRoom);
		if (diningRoom != null)
			GenerateFloor(diningRoom);
		if (bedroom != null)
			GenerateFloor(bedroom);
	}




  public void GenerateFloor(RoomIndex.FloorSection floor)
	{
		float width  = floor.Width;
		float length = floor.Length;

		GameObject floorTile = GameObject.CreatePrimitive(PrimitiveType.Cube);
		
		// Move the center so the lowest-left corner lands on SpawnPos
		// (X = width, Z = length, Y = thickness)
		floorTile.transform.localPosition = floor.SpawnPos
																			+ new Vector3(width * 0.5f, 0.05f, length * 0.5f);
		floorTile.transform.localScale = new Vector3(width, 0.1f, length);

		floorTile.GetComponent<Renderer>().material.color = Color.blueViolet;

		if(floor.Name == "LivingRoom")
		{
			floorTile.GetComponent<Renderer>().material.color = Color.green;
		}
		else if (floor.Name == "DiningRoom")
		{
			floorTile.GetComponent<Renderer>().material.color = Color.yellow;
		}
		else if (floor.Name == "Kitchen")
		{
			floorTile.GetComponent<Renderer>().material.color = Color.red;
		}
		else if (floor.Name == "Bathroom")
		{
			floorTile.GetComponent<Renderer>().material.color = Color.cyan;
		}
		else if (floor.Name == "Bedroom")
		{
			floorTile.GetComponent<Renderer>().material.color = Color.orange;
		}

	}



}

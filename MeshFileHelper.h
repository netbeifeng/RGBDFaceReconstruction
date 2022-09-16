#pragma once

#include "Eigen.h"
#include "SimpleMesh.h"

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	unsigned int nVertices = width * height;

	unsigned nFaces = 0;
	std::stringstream ssfaces;

	for (int i = 0; i < nVertices - width; i++)
	{
		if (i % width == width - 1)
			continue;

		if ((vertices + i)->position.x() != MINF && (vertices + i + width)->position.x() != MINF
			&& (vertices + i + 1)->position.x() != MINF
			&& ((vertices + i)->position - (vertices + i + width)->position).squaredNorm() <= edgeThreshold
			&& ((vertices + i + 1)->position - (vertices + i + width)->position).squaredNorm() <= edgeThreshold
			&& ((vertices + i + 1)->position - (vertices + i)->position).squaredNorm() <= edgeThreshold)
		{
			nFaces++;
			ssfaces << "3 " << i << " " << i + width << " " << i + 1 << std::endl;
		}

		if ((vertices + i + width + 1)->position.x() != MINF && (vertices + i + width)->position.x() != MINF
			&& (vertices + i + 1)->position.x() != MINF
			&& ((vertices + i + width + 1)->position - (vertices + i + width)->position).squaredNorm() <= edgeThreshold
			&& ((vertices + i + width)->position - (vertices + i + 1)->position).squaredNorm() <= edgeThreshold
			&& ((vertices + i + 1)->position - (vertices + i + width + 1)->position).squaredNorm() <= edgeThreshold)
		{
			nFaces++;
			ssfaces << "3 " << i + width << " " << i + width + 1 << " " << i + 1 << std::endl;
		}
	}


	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()){
		std::cout << "Failed to write mesh!: " << filename << "\nCheck file path!" << std::endl;
		return false;
	}
	outFile << "COFF" << std::endl; // header

	// Write numVertices numFaces numEdges
	outFile << nVertices << " " << nFaces << " 0" << std::endl;


	// Save vertices
	int i = 0, j = 0;
	int idx = 0;
	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			idx = i * width + j;
			if (vertices[idx].position.x() == MINF)
				outFile << "0.0 0.0 0.0 0 0 0 0" << endl;
			else
			{
				outFile << vertices[idx].position.x() << " " << vertices[idx].position.y() << " " << vertices[idx].position.z()
					<< " " << (int)vertices[idx].color[0] << " " << (int)vertices[idx].color[1]
					<< " " << (int)vertices[idx].color[2] << " " << (int)vertices[idx].color[3] << endl;
			}
		}
	}



	// Save valid faces
	outFile << ssfaces.rdbuf();
	outFile.close();

	return true;
}

bool WriteLandmarks(Vertex* vertices, unsigned int nVertices, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm
	
	unsigned nFaces = 0;

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;
	outFile << "COFF" << std::endl; // header

	// Write numVertices numFaces numEdges
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// Save vertices
	for (int i = 0; i < nVertices; i++)
	{
		if (vertices[i].position.x() == MINF)
			outFile << "0.0 0.0 0.0 0 0 0 0" << endl;
		else
		{
			outFile << vertices[i].position.x() << " " << vertices[i].position.y() << " " << vertices[i].position.z()
				<< " " << (int)vertices[i].color[0] << " " << (int)vertices[i].color[1]
				<< " " << (int)vertices[i].color[2] << " " << (int)vertices[i].color[3] << endl;
		}
	}

	outFile.close();

	return true;
}
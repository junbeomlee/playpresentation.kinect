#include "stdafx.h"
#include "handJoint.h"
#include <string>
void handJoint::loadData(ifstream &inFile)
{
	for (int i = 0; i < 21; i++)
	{
		double x,y,z;
		inFile >> x;
		inFile >> y;
		inFile >> z;
		(*this).push_back(joint(x,y,-z + 130));
	}

}

void handJoint::printData()
{
	string fingerName[21] = { "wrist", "index_mcp", "index_pip", "index_dip", "indepx_tip", "middle_mcp", "middle_pip", "middle_dip", "middle_tip", "ring_mcp", "ring_pip",
		"ring_dip", "ring_tip", "little_mcp", "little_pip", "little_dip", "little_tip", "thumb_mcp", "thumb_pip", "thumb_dip", "thumb_tip" };
	for (int i = 0; i < 21; i++)
	{
		cout << fingerName[i] << " : " << (*this)[i].x << ", " << (*this)[i].y << ", " << (*this)[i].z << endl;
	}
}
//0. wrist
//1. index_mcp
//2. index_pip
//3. index_dip
//4. index_tip
//5. middle_mcp
//6. middle_pip
//7. middle_dip
//8. middle_tip
//9. ring_mcp
//10. ring_pip
//11. ring_dip
//12. ring_tip
//13. little_mcp
//14. little_pip
//15. little_dip
//16. little_tip
//17. thumb_mcp
//18. thumb_pip
//19. thumb_dip
//20. thumb_tip
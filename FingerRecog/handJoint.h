#ifndef HANDJOINT_H
#define HANDJOINT_H
#include "stdafx.h"
#include "joint.h"
#include <vector>
#include "std.h"
using namespace std;
//0. wrist
//1. index_mcp -> 4n-3;
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
class handJoint : public vector < joint >
{
public:
	void loadData(ifstream &inFile);
	void printData();
};

#endif
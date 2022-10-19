//
// Created by Peter on 9/26/2022.
// Currently Unused
//

#include "../inc/pNode.h"



pNode::pNode(int id, int p) {
    id = id;
    nPriority = p;
}

bool operator<(const pNode& a, const pNode& b)
{
    return a.nPriority < b.nPriority;
}


//
// Created by peter on 9/26/2022.
// Currently Unused
//

#ifndef MINLATENCYWSN_PNODE_H
#define MINLATENCYWSN_PNODE_H


class pNode {
public:
    int nID;
    int nPriority;

    pNode(int id, int p);

    friend bool operator<(const pNode& a, const pNode& b);

};




#endif //MINLATENCYWSN_PNODE_H

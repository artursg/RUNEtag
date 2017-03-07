#ifndef TAG_CODE_H_
#define TAG_CODE_H_

#include <vector>
#include <iostream>

class TagCode
{
public:
    TagCode(void);
    ~TagCode(void);

    std::vector<long> code;
    std::vector<bool> bcode;
    long idx;

    static bool load_tags( std::istream& ifs, std::vector<TagCode>& out_tags );
};


#endif
#include "TagCode.h"
#include "coding.h"

TagCode::TagCode(void)
{
}

TagCode::~TagCode(void)
{
}


bool TagCode::load_tags( std::istream& ifs, std::vector<TagCode>& out_tags )
{
    size_t num_tags;
    ifs >> num_tags;
    if( ifs.fail() )
        return false;

    out_tags.resize( num_tags );

    for( size_t i=0; i<num_tags; ++i )
    {
        ifs >> out_tags[i].idx;

        size_t codelen;
        ifs>>codelen;

        out_tags[i].code.resize(codelen);
        for( size_t j=0; j<codelen; ++j )
        {
            ifs >> out_tags[i].code[j];
            if( ifs.fail() )
                return false;
        }
        out_tags[i].bcode = cv::runetag::Coding::unpack( out_tags[i].code );
    }

    return true;
}

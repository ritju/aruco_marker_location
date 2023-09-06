

#include <openni2/OpenNI.h>
#include <openni2/PS1080.h>
#include <iostream>

using namespace std;

int main()
{
        cout << "************** Test **************" << endl;
        openni::Status rc;
        rc = openni::OpenNI::initialize();
        if(rc != openni::STATUS_OK)
        {
                cout << "initialize failed." << endl;
        }


        cout << "************** End **************" << endl;
}
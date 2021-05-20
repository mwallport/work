// file test.cpp
#include <map>
#include <iomanip>
#include <inttypes.h>
#include "menu.h"


controlProtocol* pCP;               //gobal controlProtocol pointer
map<int, menuItemBase*> menuItems;  // gobal map of the menu items


void createMenu(void);              // create the menu
void handleMenu(void);              // execute the menu
void showMenu(void);                // put menu on the screen
void executeMenuItem(void);         // get user input and execute command

int main(int argc, char** argv)
{
    uint16_t        speed;
    

    if( (3 > argc) )
    {
        printf("Please supply USB device and speed\n");
        printf("i.e. test.exe COM6 57600\n");
        return(-1);
    }
    
    sscanf(argv[2], "%u", &speed);             // assuming this works
    printf("trying usb: %s, speed %u\n", argv[1], speed);
    
    #ifdef __USING_WINDOWS_USB__
    WORD wVersionRequested;
    WSADATA wsaData;
    int err;

    /* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
    wVersionRequested = MAKEWORD(2, 2);

    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0) {
        /* Tell the user that we could not find a usable */
        /* Winsock DLL.                                  */
        printf("WSAStartup failed with error: %d\n", err);
        return 1;
    }
    #endif

    // my address, peer address, usb file
    pCP = new controlProtocol(0, 1, argv[1], speed);

    createMenu();
    handleMenu();
       
    return(0);
}


void createMenu(void)
{
    // create and push all the menu items into the menuItems map
    menuItemBase*   p_menuItem;
    int i = 1;


    p_menuItem  = new menuStartUpCmd;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuShutDownCmd;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetStatus;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetTempCmd;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetHumidity;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuSetHumidityThreshold;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetHumidityThreshold;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuStartChiller;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuStopChiller;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

/*  5/19/2021 - using PP commands on Huber chiller, which do not
    support get chiller info..
    p_menuItem  = new menuGetChillerInfo;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem)); */

    p_menuItem  = new menuSetChillerTemperature;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetChillerTemperature;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetChillerObjTemperature;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetTECInfo;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuSetTECTemperature;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetTECTemperature;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetTECObjTemperature;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuEnableTECs;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuDisableTECs;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuSetRTCCmd;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetRTCCmd;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuClrEventLogCmd;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));

    p_menuItem  = new menuGetEventLogCmd;
    menuItems.insert(pair<int, menuItemBase*>(i++, p_menuItem));
}


void handleMenu(void)
{
    do
    {
        showMenu();
        executeMenuItem();
    } while(true);  // until user presses Ctrl+C to kill application
}


void showMenu(void)
{
    cout << endl;; cout.flush();
    
    for(map<int, menuItemBase*>::iterator i = menuItems.begin(); i != menuItems.end(); ++i)
    {
        string item(to_string(i->first));
        item.append(".");
        cout << setw(5) << std::left << item << *(i->second) << endl;
    }
    
    cout.flush(); cout << "\nEnter number (Ctrl+C to exit): "; cout.flush();
}


void executeMenuItem(void)
{
    int selection;
    map<int, menuItemBase*>::iterator item;
    
    
    cin >> selection;
    item = menuItems.find(selection);
    if(item != menuItems.end())
    {
        // get parameters for the selection
        (item->second)->getParameters();

        // exectue the command
        (item->second)->execute(pCP);
        
    } else
        cout << selection << " is not valid" << endl; 

    cin.clear(); cin.ignore(1000, '\n');
           
    cout << "\nPress Enter key to continue";
    getchar();
}


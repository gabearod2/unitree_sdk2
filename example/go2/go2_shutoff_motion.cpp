#include <iostream>
#include <string>
#include <unistd.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::robot;
using namespace unitree::robot::b2;

static int deactivate_motion(MotionSwitcherClient& msc, int max_tries = 6, int sleep_sec = 1)
{
    for (int k = 0; k < max_tries; ++k)
    {
        std::string form, name;
        int32_t ret = msc.CheckMode(form, name);
        if (ret != 0)
        {
            std::cerr << "[ERROR] CheckMode failed: " << ret << "\n";
            // still attempt ReleaseMode in case CheckMode is flaky
        }
        else
        {
            if (name.empty())
            {
                std::cout << "[OK] Motion service already deactivated (mode name empty).\n";
                return 0;
            }
            std::cout << "[INFO] Current mode: form=" << form << " name=" << name << "\n";
        }

        std::cout << "[INFO] Calling ReleaseMode()...\n";
        ret = msc.ReleaseMode();
        if (ret == 0)
        {
            std::cout << "[OK] ReleaseMode succeeded.\n";
            // Verify it actually cleared
            std::string form2, name2;
            int32_t ret2 = msc.CheckMode(form2, name2);
            if (ret2 == 0 && name2.empty())
            {
                std::cout << "[OK] Verified motion service deactivated.\n";
                return 0;
            }
            std::cout << "[WARN] ReleaseMode returned 0 but mode still appears active; retrying...\n";
        }
        else
        {
            std::cerr << "[WARN] ReleaseMode failed: " << ret << " (retrying)\n";
        }

        sleep(sleep_sec);
    }

    return 1;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <networkInterface>\n"
                  << "Example: " << argv[0] << " enp2s0\n";
        return 2;
    }

    const char* netif = argv[1];

    std::cout << "WARNING: This will deactivate Unitree motion control service.\n"
              << "Make sure the robot is in a safe state (supported / on ground).\n"
              << "Press Enter to continue...\n";
    std::cin.get();

    // Init Unitree DDS channel factory (needed for SDK RPC)
    ChannelFactory::Instance()->Init(0, netif);

    MotionSwitcherClient msc;
    msc.SetTimeout(10.0f);
    msc.Init();

    int rc = deactivate_motion(msc);
    if (rc == 0)
    {
        std::cout << "[DONE] Motion deactivated.\n";
        return 0;
    }

    int32_t sret = msc.SetSilent(true);
    std::cout << "SetSilent(true) ret=" << sret << "\n";

    std::cerr << "[FAILED] Could not deactivate motion after retries.\n";
    return 1;
}

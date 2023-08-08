#include <Sts1CobcSw/EduCommunicationErrorThread.hpp>
#include <Sts1CobcSw/EduProgramQueueThread.hpp>
#include <Sts1CobcSw/ThreadPriorities.hpp>
#include <Sts1CobcSw/TopicsAndSubscribers.hpp>

#include <rodos_no_using_namespace.h>


namespace sts1cobcsw
{
constexpr auto stackSize = 2'000U;
constexpr auto eduShutDownDelay = 2 * RODOS::SECONDS;
std::int32_t eduCommunicationErrorCounter = 0;


class EduCommunicationErrorThread : public RODOS::StaticThread<stackSize>
{
public:
    EduCommunicationErrorThread()
        : StaticThread("EduCommunicationThread", eduCommunicationErrorThreadPriority)
    {
    }

private:
    void init() override
    {
    }


    void run() override
    {
        while(true)
        {
            RODOS::AT(RODOS::END_OF_TIME);
            // RODOS::PRINTF("EduCommunicationThread resumed in main loop\n");

            eduCommunicationErrorCounter++;

            RODOS::PRINTF("[EduCommunicationErrorThread] Resetting the Edu\n");
            // Reset EDU
            edu.TurnOff();
            RODOS::AT(RODOS::NOW() + eduShutDownDelay);
            edu.TurnOn();

            //
            [[maybe_unused]] auto status = edu.GetStatus();

            // Busy wait
            RODOS::PRINTF("[EduCommunicationErrorThread] Entering busy wait\n");
            auto eduIsAlive = false;
            while(not eduIsAlive)
            {
                yield();  // Force recalculation of scheduling!
                eduIsAliveBufferForCommunicationError.get(eduIsAlive);
            }
            RODOS::PRINTF("[EduCommunicationErrorThread] Leaving busy wait\n");
        }
    }
} eduCommunicationErrorThread;

auto ResumeEduErrorCommunicationThread() -> void
{
    eduCommunicationErrorThread.resume();
    RODOS::PRINTF("[EduCommunicationErrorThread] EduCommunicationThread resumed\n");
}
}

#include <Sts1CobcSw/EduProgramStatusHistory.hpp>

#include <rodos.h>


namespace sts1cobcsw
{

auto FindStatusAndHistoryEntry(std::uint16_t programId, std::uint16_t queueId) -> StatusHistoryEntry
{
    auto counter = 0;
    auto statusHistoryEntry = StatusHistoryEntry{};
    do
    {
        statusHistory.get(statusHistoryEntry);
        // RODOS::PRINTF("%d,%d vs %d,%d\n", statusHistoryEntry.programId,
        // statusHistoryEntry.queueId, programId, queueId);
    } while(statusHistoryEntry.queueId != queueId or statusHistoryEntry.programId != programId);

    return statusHistoryEntry;
}
}

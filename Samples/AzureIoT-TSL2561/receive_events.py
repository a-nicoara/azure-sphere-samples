import asyncio
from azure.eventhub.aio import EventHubConsumerClient

async def on_event(partition_context, event):
    # Print the event data.
    print("Received the event: \"{}\" from the partition with ID: \"{}\"".format(event, partition_context.partition_id))

    # Update the checkpoint so that the program doesn't read the events
    # that it has already read when you run it next time.
    await partition_context.update_checkpoint(event)

async def main():
    # Create a consumer client for the event hub.
    client = EventHubConsumerClient.from_connection_string("Endpoint=sb://hslu-sphere-eventhub.servicebus.windows.net/;SharedAccessKeyName=subscriberapp;SharedAccessKey=Ga5/QiNJxbRWigXRE3INyVt5NH2VbUBeYMCI0iOrmQU=", consumer_group="$Default", eventhub_name="spherelight")
    async with client:
        # Call the receive method.
        await client.receive(on_event=on_event)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    # Run the main method.
    loop.run_until_complete(main())

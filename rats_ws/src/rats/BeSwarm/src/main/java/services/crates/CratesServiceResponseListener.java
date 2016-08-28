package services.crates;

import org.ros.exception.RemoteException;
import org.ros.internal.message.Message;
import org.ros.node.service.ServiceResponseListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.CountDownLatch;

/** @author Hoang Tung Dinh */
final class CratesServiceResponseListener<T extends Message> implements ServiceResponseListener<T> {

  private static final Logger logger = LoggerFactory.getLogger(CratesServiceResponseListener.class);
  private final CountDownLatch countDownLatch;

  private CratesServiceResponseListener(CountDownLatch countDownLatch) {
    this.countDownLatch = countDownLatch;
  }

  /**
   * Creates a service response listener for the Crates simulator.
   *
   * @param countDownLatch the countdown latch for thread synchronization
   * @param <T> the response message type
   * @return a new instance of the service response listener
   */
  public static <T extends Message> CratesServiceResponseListener<T> create(
      CountDownLatch countDownLatch) {
    return new CratesServiceResponseListener<>(countDownLatch);
  }

  @Override
  public void onSuccess(T t) {
    logger.trace("Successfully sent message!!!");
    countDownLatch.countDown();
  }

  @Override
  public void onFailure(RemoteException e) {
    logger.info("Cannot send message!!!");
    countDownLatch.countDown();
  }
}

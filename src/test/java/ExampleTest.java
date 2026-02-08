package test.java;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ExampleTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  @BeforeEach // this method will run before each test
  void setup() {
    assert 1 == 1;
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    System.out.println("Onto next test");
  }

  @Test
  void exampleTest() {
    assertEquals(1, 1);
  }
}

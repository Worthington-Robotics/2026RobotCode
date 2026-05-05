package frc.WorBots.util.debug;

public class LoggingTools {
    public class RollingAverageLogger {
      private double value = 0.0;
      private int entries = 0;
      private String table;
      private String key;
      public RollingAverageLogger(String table, String key){
        this.table = table;
        this.key = key;
      }

      public void addValue(double value){
          this.value += value;
          entries++;
          NTLogger.putNumber(table, key, this.value / entries);
      }

      public double get(){
        return value / entries;
      }

      public void reset(){
        value = 0;
        entries = 0;
      }
  }
}

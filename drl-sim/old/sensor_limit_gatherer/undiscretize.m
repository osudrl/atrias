function val = undiscretize(count, val_min, val_max, count_min, count_max)
  val = (count - count_min) * (val_max - val_min) / (count_max - count_min) + val_min;
end
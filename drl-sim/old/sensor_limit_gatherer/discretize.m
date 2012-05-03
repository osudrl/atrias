function count = discretize(val, val_min, val_max, count_min, count_max)
  count = round( (val - val_min) / (val_max - val_min) * (count_max - count_min) + count_min );
end
# Inputs
cmd_speed = 0.5        # Positive speed (moving toward upper limit)
current_pos = 95       # Near the limit
upper_limit = 100

# 1. Check if we are violating the limit (True/False)
# 2. Convert to int (1 or 0)
# 3. Use logic: "Allow speed only if (pos < limit OR speed is negative)"
can_move = int(current_pos < upper_limit or cmd_speed < 0)

# Apply the mask
final_speed = cmd_speed * can_move

print(final_speed) # Result: 0.0 (Stopped because it's past the limit)
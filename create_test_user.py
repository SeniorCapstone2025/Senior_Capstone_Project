#!/usr/bin/env python3
"""
Quick script to create a test user for the dashboard.
Run: python create_test_user.py
"""

import sys
import bcrypt
from app.database import create_user, get_user_by_username

def create_test_user(username: str = "admin", password: str = "admin1234", is_admin: bool = True):
    """Create a test user for authentication."""
    
    # Check if user already exists
    existing = get_user_by_username(username)
    if existing:
        print(f"✗ User '{username}' already exists!")
        return False
    
    # Hash password
    password_hash = bcrypt.hashpw(password.encode("utf-8"), bcrypt.gensalt()).decode("utf-8")
    
    try:
        user = create_user(username, password_hash, is_admin=is_admin)
        print(f"✓ Test user created successfully!")
        print(f"  Username: {username}")
        print(f"  Password: {password}")
        print(f"  Admin:    {is_admin}")
        print(f"\n  Use these credentials to log into the dashboard at http://localhost:3000")
        return True
    except Exception as e:
        print(f"✗ Error creating user: {e}")
        return False

if __name__ == "__main__":
    # Allow custom username/password via command line
    username = sys.argv[1] if len(sys.argv) > 1 else "admin"
    password = sys.argv[2] if len(sys.argv) > 2 else "admin1234"
    is_admin = sys.argv[3].lower() != "false" if len(sys.argv) > 3 else True
    
    success = create_test_user(username, password, is_admin)
    sys.exit(0 if success else 1)

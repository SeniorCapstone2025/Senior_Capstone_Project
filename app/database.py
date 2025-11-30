import mysql.connector
from mysql.connector import Error

def get_db_connection():
    try:
        connection = mysql.connector.connect(host="localhost", user="root", password="Tashauna2", database="rover")
        return connection

    except Error as e:
            print(f"Database connection error: {e}")
            return None

def save_command(command,status):
    conn = get_db_connection()
    if conn is None:
        return

    cursor = conn.cursor()
    query = "INSERT INTO command_logs (command,status) VALUES (%s,%s)"
    cursor.execute(query, (command,status))

    conn.commit()
    conn.close()
    cursor.close()

def save_status(state,battery,current_task):
    conn = get_db_connection()
    if conn is None:
        return

    cursor = conn.cursor()
    query = "INSERT INTO status_logs (state,battery,current_task) VALUES (%s,%s,%s)"
    cursor.execute(query, (state,battery,current_task))

    conn.commit()
    conn.close()
    cursor.close()

def save_detected_object(object_name,confidence,x,y):
    conn = get_db_connection()
    if conn is None:
        return

    cursor = conn.cursor()
    query = """
        INSERT INTO detections (object_name, confidence, x, y)
        VALUES (%s, %s, %s, %s)
    """
    cursor.execute(query, (object_name,confidence,x,y))

    conn.commit()
    conn.close()
    cursor.close()
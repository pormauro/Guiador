import os

def main():
    # Directorio donde está el script
    base_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(base_dir)

    print(f"Carpeta base:\n{base_dir}\n")

    output_filename = "resultado_unificado.txt"
    script_path = os.path.abspath(__file__)
    output_path = os.path.join(base_dir, output_filename)

    print("Iniciando búsqueda recursiva...\n")

    with open(output_path, "w", encoding="utf-8") as out:
        for root, dirs, files in os.walk(base_dir):

            # ✨ Evitar completamente la carpeta .git
            if ".git" in root.replace("\\", "/").split("/"):
                continue

            # También evitamos que os.walk ENTRE a .git si aparece en dirs
            if ".git" in dirs:
                dirs.remove(".git")

            for filename in files:
                filepath = os.path.join(root, filename)

                # Saltar el propio script y el archivo de salida
                if os.path.abspath(filepath) in (script_path, output_path):
                    print(f"[SKIP] {filepath} (script o salida)")
                    continue

                try:
                    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                        rel_path = os.path.relpath(filepath, base_dir)
                        print(f"[OK] Copiando: {rel_path}")

                        out.write(f"===== ARCHIVO: {rel_path} =====\n")
                        out.write(f.read())
                        out.write("\n\n")
                except Exception as e:
                    print(f"[ERROR] No se pudo leer {filepath}: {e}")

    print("\n✔ Proceso finalizado.")
    print(f"✔ Archivo generado: {output_path}\n")
    input("Presiona ENTER para salir...")

if __name__ == "__main__":
    main()

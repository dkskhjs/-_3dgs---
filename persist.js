// Files added to the page are kept in this browser (IndexedDB) and opened again on the next visit,
// together with where each layer was placed. Nothing leaves the browser; every call fails quietly when
// storage is unavailable (private window, blocked site data).
const DB = "zzikbang-scan", VER = 1;
let dbp = null;
function db() {
  if (!dbp) dbp = new Promise((resolve) => {
    try {
      const req = indexedDB.open(DB, VER);
      req.onupgradeneeded = () => {
        req.result.createObjectStore("files", { keyPath: "id" }); // the file itself (written once)
        req.result.createObjectStore("meta", { keyPath: "id" }); // placement, rewritten as it changes
      };
      req.onsuccess = () => resolve(req.result);
      req.onerror = () => resolve(null);
    } catch {
      resolve(null);
    }
  });
  return dbp;
}
async function tx(stores, mode, fn) {
  const d = await db();
  if (!d) return null;
  return new Promise((resolve) => {
    try {
      const t = d.transaction(stores, mode);
      const out = fn(...stores.map((s) => t.objectStore(s)));
      t.oncomplete = () => resolve(out?.result ?? out ?? true);
      t.onerror = t.onabort = () => resolve(null);
    } catch {
      resolve(null);
    }
  });
}
export const newPersistId = () => `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
export function persistFile(id, file, meta) {
  return tx(["files", "meta"], "readwrite", (files, metas) => {
    files.put({ id, blob: file, name: file.name, type: file.type, lastModified: file.lastModified });
    metas.put({ ...meta, id });
  });
}
export function persistMeta(metaList) {
  return tx(["meta"], "readwrite", (metas) => { for (const m of metaList) metas.put(m); });
}
export function persistDelete(id) {
  return tx(["files", "meta"], "readwrite", (files, metas) => { files.delete(id); metas.delete(id); });
}
/** [{ meta, file }] in the order they were added */
export async function persistLoadAll() {
  const d = await db();
  if (!d) return [];
  const all = (store) => tx([store], "readonly", (s) => s.getAll());
  const [files, metas] = [await all("files"), await all("meta")];
  if (!files || !metas) return [];
  const byId = new Map(files.map((f) => [f.id, f]));
  return metas
    .filter((m) => byId.has(m.id))
    .sort((a, b) => (a.order ?? 0) - (b.order ?? 0))
    .map((meta) => {
      const f = byId.get(meta.id);
      return { meta, file: new File([f.blob], f.name, { type: f.type, lastModified: f.lastModified }) };
    });
}
